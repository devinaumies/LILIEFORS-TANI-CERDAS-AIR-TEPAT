/*
  TaniCerdas - AirTepat
  Sistem prioritas penyiraman berbasis anggaran air untuk petani kecil
  Samsung Innovation Campus Batch 8 - Selection Phase 2 (IoT: Device & Hardware Discovery)

  Alur: SENSOR (2x tanah, 1x tandon) -> ESP32 / LOGIC -> OUTPUT (relay pompa, LED, buzzer)
        -> SERIAL / MQTT (HiveMQ Cloud) -> PYTHON DASHBOARD

  Simulasi Wokwi: sensor kelembapan tanah disimulasikan dengan potensiometer (ADC),
  pompa dengan modul relay + LED, tandon dengan sensor ultrasonik HC-SR04.

  Perintah (Serial Monitor ATAU topik MQTT tanicerdas/pump/command):
    AUTO    -> mode otomatis (pompa jalan sesuai rekomendasi jika semua kondisi aman)
    MANUAL  -> mode rekomendasi saja (default)
    STOP    -> hentikan pompa dan kembali MANUAL
    PUMP_A / PUMP_B -> siram petak secara manual (tetap melewati pemeriksaan keamanan)
*/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "engine.h"

// ================= KONFIGURASI JARINGAN =================
const char* WIFI_SSID    = "Wokwi-GUEST";   // WiFi gratis bawaan Wokwi
const char* WIFI_PASS    = "";
const int   WIFI_CHANNEL = 6;
const char* MQTT_HOST    = "GANTI_HOST.s1.eu.hivemq.cloud";  // <-- isi dari HiveMQ Cloud
const uint16_t MQTT_PORT = 8883;                             // TLS
const char* MQTT_USER    = "GANTI_USERNAME";                 // <-- isi dari HiveMQ Cloud
const char* MQTT_PASS    = "GANTI_PASSWORD";                 // <-- isi dari HiveMQ Cloud

// Topik MQTT (sesuai proposal)
const char* TOPIC_PLOT_A   = "tanicerdas/plotA/sensor";
const char* TOPIC_PLOT_B   = "tanicerdas/plotB/sensor";
const char* TOPIC_TANK     = "tanicerdas/tank/status";
const char* TOPIC_RECO     = "tanicerdas/recommendation";
const char* TOPIC_PUMP_CMD = "tanicerdas/pump/command";
const char* TOPIC_PUMP_STA = "tanicerdas/pump/status";

// ================= PIN GPIO =================
const int PIN_SOIL_A   = 36;  // ADC1_CH0 (input-only) - sensor tanah petak A (potensiometer)
const int PIN_SOIL_B   = 39;  // ADC1_CH3 (input-only) - sensor tanah petak B (potensiometer)
const int PIN_TRIG     = 5;   // HC-SR04 TRIG
const int PIN_ECHO     = 18;  // HC-SR04 ECHO
const int PIN_RELAY    = 26;  // IN modul relay (pompa)
const int PIN_LED_A    = 16;  // indikator rekomendasi/penyiraman petak A
const int PIN_LED_B    = 17;  // indikator rekomendasi/penyiraman petak B
const int PIN_LED_ALRT = 27;  // LED merah peringatan
const int PIN_BUZZER   = 25;  // buzzer peringatan
const int PIN_BTN_STOP = 14;  // tombol emergency stop (INPUT_PULLUP, aktif LOW)

const bool RELAY_ACTIVE_LOW = false;  // ubah ke true jika relay aktif-LOW

// ================= KALIBRASI & PARAMETER =================
// Sensor kapasitif nyata: nilai ADC makin tinggi = makin kering. Wokwi: putar potensiometer.
const int   ADC_DRY = 3300;           // ADC saat kering (0 %)
const int   ADC_WET = 1300;           // ADC saat basah  (100 %)
const int   ADC_VALID_MIN = 150;      // di luar rentang ini = sensor putus/korslet -> ERROR
const int   ADC_VALID_MAX = 3950;

const float TANK_FULL_CM  = 2.0f;     // jarak sensor ke permukaan saat penuh
const float TANK_EMPTY_CM = 22.0f;    // jarak sensor ke permukaan saat kosong
const float TANK_CAP_ML   = 1000.0f;  // kapasitas tandon

const float FLOW_ML_PER_S = 25.0f;    // debit pompa (hasil kalibrasi wadah ukur; ganti dengan nilai nyata)
const unsigned long PUMP_MAX_MS = 30000UL;   // batas waktu pompa (fail-safe timeout)
const unsigned long SOAK_MS     = 60000UL;   // waktu resapan setelah penyiraman (detik nyata di simulasi)
const float SIM_MIN_PER_SEC     = 1.0f;      // 1 detik nyata = 1 menit simulasi (untuk komponen T)

const unsigned long CYCLE_MS   = 2000UL;     // siklus baca + keputusan
const unsigned long PUBLISH_MS = 5000UL;     // siklus kirim MQTT

PlotConfig CFG[N_PLOTS] = {
  {'A', "Cabai",    40.0f, 80.0f, 400.0f},   // id, tanaman, target/ambang (%), urgensi U, kebutuhan (ml)
  {'B', "Kangkung", 45.0f, 50.0f, 350.0f}
};
EngineParams PARAMS = {120.0f, 100.0f};      // T_max (menit simulasi), cadangan tandon (ml)

// ================= STATE =================
WiFiClientSecure netClient;
PubSubClient mqtt(netClient);

int   gRaw[N_PLOTS] = {0, 0};
float gMoist[N_PLOTS] = {0, 0};
bool  gValid[N_PLOTS] = {false, false};
float gTankMl = 0, gTankCm = 0; bool gTankValid = false;
EngineResult gRes;
char  gReason[170] = "";

unsigned long lastWaterMs[N_PLOTS] = {0, 0};
unsigned long soakUntilMs[N_PLOTS] = {0, 0};

bool modeAuto = false;
bool estop = false;
bool pumpOn = false;
int  pumpTarget = -1;
unsigned long pumpStartMs = 0, pumpDurMs = 0;
float waterUsedTotalMl = 0;

unsigned long lastCycleMs = 0, lastPublishMs = 0, lastMqttTryMs = 0;
bool btnLast = HIGH, btnStable = HIGH; unsigned long btnChangeMs = 0;
char serialBuf[24]; int serialLen = 0;

// ================= PROTOTIPE =================
void handleCommand(char* cmd, const char* src);
void startPump(int idx, const char* why);
void stopPump(const char* why);
void publishAll(bool force);
void publishPumpStatus(const char* event);

// ================= SENSOR =================
void readSoil(int pin, int idx) {
  long sum = 0;
  for (int k = 0; k < 10; k++) { sum += analogRead(pin); delayMicroseconds(200); }
  int raw = (int)(sum / 10);
  gRaw[idx] = raw;
  gValid[idx] = (raw >= ADC_VALID_MIN && raw <= ADC_VALID_MAX);
  float pct = 100.0f * (float)(ADC_DRY - raw) / (float)(ADC_DRY - ADC_WET);
  gMoist[idx] = clampf(pct, 0.0f, 100.0f);
}

void readTank() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) { gTankValid = false; gTankCm = 0; gTankMl = 0; return; }
  gTankCm = (float)us * 0.0343f / 2.0f;
  gTankValid = (gTankCm >= 2.0f && gTankCm <= 100.0f);
  float frac = (TANK_EMPTY_CM - gTankCm) / (TANK_EMPTY_CM - TANK_FULL_CM);
  gTankMl = clampf(frac, 0.0f, 1.0f) * TANK_CAP_ML;
}

// ================= POMPA =================
void setPump(bool on) { digitalWrite(PIN_RELAY, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW); }

bool pumpSafeNow() { return gRes.sys == SYS_OK && !estop; }

void startPump(int idx, const char* why) {
  if (pumpOn || idx < 0) return;
  unsigned long dur = (unsigned long)(CFG[idx].volumeMl / FLOW_ML_PER_S * 1000.0f);
  if (dur > PUMP_MAX_MS) dur = PUMP_MAX_MS;
  pumpTarget = idx; pumpStartMs = millis(); pumpDurMs = dur; pumpOn = true;
  setPump(true);
  Serial.printf(">> POMPA ON  petak %c (%s) | %.0f ml | %lu ms | sumber=%s\n",
                CFG[idx].id, CFG[idx].crop, CFG[idx].volumeMl, dur, why);
  publishPumpStatus(why);
}

void stopPump(const char* why) {
  if (!pumpOn) return;
  setPump(false);
  unsigned long el = millis() - pumpStartMs;
  float added = (float)el / 1000.0f * FLOW_ML_PER_S;
  if (pumpTarget >= 0 && el > 1000UL) {
    waterUsedTotalMl += added;
    lastWaterMs[pumpTarget] = millis();
    soakUntilMs[pumpTarget] = millis() + SOAK_MS;
  }
  Serial.printf(">> POMPA OFF petak %c | durasi %lu ms | est. %.0f ml | alasan=%s\n",
                pumpTarget >= 0 ? CFG[pumpTarget].id : '-', el, added, why);
  pumpOn = false; pumpTarget = -1;
  publishPumpStatus(why);
}

void pumpUpdate(unsigned long now) {
  if (!pumpOn) return;
  if (now - pumpStartMs >= pumpDurMs) { stopPump(pumpDurMs >= PUMP_MAX_MS ? "TIMEOUT" : "SELESAI"); }
}

// ================= KEPUTUSAN =================
void buildReason() {
  if (gRes.sys == SYS_SENSOR_ERROR) { snprintf(gReason, sizeof(gReason), "Sensor tanah tidak valid; pompa dimatikan"); return; }
  if (gRes.sys == SYS_TANK_ERROR)   { snprintf(gReason, sizeof(gReason), "Sensor tandon tidak valid; pompa dimatikan"); return; }
  if (gRes.sys == SYS_TANK_LOW)     { snprintf(gReason, sizeof(gReason), "Air tandon %.0f ml di bawah batas aman %.0f ml", gTankMl, PARAMS.reserveMl); return; }
  if (estop)                        { snprintf(gReason, sizeof(gReason), "Emergency stop aktif; pompa dikunci"); return; }
  if (gRes.nCand == 0)              { snprintf(gReason, sizeof(gReason), "Semua petak cukup lembap atau sedang resapan"); return; }
  int i = gRes.order[0];
  if (gRes.plot[i].state == ST_WATER)
    snprintf(gReason, sizeof(gReason), "Prioritas 1 petak %c (Q=%.1f). Air %.0f ml cukup untuk %.0f ml",
             CFG[i].id, gRes.plot[i].Q, gTankMl - PARAMS.reserveMl, CFG[i].volumeMl);
  else
    snprintf(gReason, sizeof(gReason), "Prioritas 1 petak %c (Q=%.1f). Air tidak cukup untuk %.0f ml",
             CFG[i].id, gRes.plot[i].Q, CFG[i].volumeMl);
}

void runCycle() {
  unsigned long now = millis();
  readSoil(PIN_SOIL_A, 0);
  readSoil(PIN_SOIL_B, 1);
  readTank();

  PlotInput in[N_PLOTS];
  for (int i = 0; i < N_PLOTS; i++) {
    in[i].moisture = gMoist[i];
    in[i].valid = gValid[i];
    in[i].minutesSince = (float)(now - lastWaterMs[i]) / 1000.0f * SIM_MIN_PER_SEC;
    in[i].inSoak = (now < soakUntilMs[i]);
  }
  gRes = runEngine(CFG, in, gTankMl, gTankValid, PARAMS);
  buildReason();

  // Fail-safe lokal (tidak bergantung pada Wi-Fi/MQTT/dashboard)
  if (pumpOn && !pumpSafeNow()) stopPump("FAILSAFE");
  if (modeAuto && !pumpOn && pumpSafeNow() && gRes.firstWater >= 0) startPump(gRes.firstWater, "AUTO");

  // Serial Monitor
  Serial.printf("\n[t=%4lus] MODE=%s SYS=%s TANDON=%.0f ml (%.1f cm%s) POMPA=%s%s\n",
                now / 1000UL, modeAuto ? "AUTO" : "MANUAL", sysName(gRes.sys), gTankMl, gTankCm,
                gTankValid ? "" : " INVALID", pumpOn ? "ON" : "OFF", estop ? " [ESTOP]" : "");
  for (int i = 0; i < N_PLOTS; i++) {
    Serial.printf("  %c %-8s raw=%4d M=%5.1f%% tgt=%2.0f | D=%5.1f U=%3.0f T=%5.1f Q=%5.1f | %s%s\n",
                  CFG[i].id, CFG[i].crop, gRaw[i], gMoist[i], CFG[i].target,
                  gRes.plot[i].D, gRes.plot[i].U, gRes.plot[i].T, gRes.plot[i].Q,
                  stateName(gRes.plot[i].state), gValid[i] ? "" : " (sensor invalid)");
  }
  Serial.printf("  Alasan: %s\n", gReason);
}

// ================= KOMUNIKASI MQTT =================
void publishPumpStatus(const char* event) {
  if (!mqtt.connected()) return;
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"uptime_s\":%lu,\"pump\":\"%s\",\"target\":\"%c\",\"mode\":\"%s\",\"event\":\"%s\",\"water_used_ml\":%.0f,\"estop\":%s}",
    millis() / 1000UL, pumpOn ? "ON" : "OFF", pumpTarget >= 0 ? CFG[pumpTarget].id : '-',
    modeAuto ? "AUTO" : "MANUAL", event, waterUsedTotalMl, estop ? "true" : "false");
  mqtt.publish(TOPIC_PUMP_STA, buf);
}

void publishAll(bool force) {
  unsigned long now = millis();
  if (!force && now - lastPublishMs < PUBLISH_MS) return;
  lastPublishMs = now;
  if (!mqtt.connected()) return;
  char buf[900];
  unsigned long up = now / 1000UL;

  for (int i = 0; i < N_PLOTS; i++) {
    float mins = (float)(now - lastWaterMs[i]) / 1000.0f * SIM_MIN_PER_SEC;
    snprintf(buf, sizeof(buf),
      "{\"plot_id\":\"%c\",\"crop\":\"%s\",\"uptime_s\":%lu,\"raw_adc\":%d,\"soil_moisture_index\":%.1f,"
      "\"target\":%.0f,\"urgency\":%.0f,\"minutes_since_watering\":%.1f,\"sensor_status\":\"%s\"}",
      CFG[i].id, CFG[i].crop, up, gRaw[i], gMoist[i], CFG[i].target, CFG[i].urgency, mins,
      gValid[i] ? "OK" : "ERROR");
    mqtt.publish(i == 0 ? TOPIC_PLOT_A : TOPIC_PLOT_B, buf);
  }

  snprintf(buf, sizeof(buf), "{\"uptime_s\":%lu,\"distance_cm\":%.1f,\"tank_ml\":%.0f,\"status\":\"%s\"}",
           up, gTankCm, gTankMl, !gTankValid ? "ERROR" : (gTankMl < PARAMS.reserveMl ? "LOW" : "OK"));
  mqtt.publish(TOPIC_TANK, buf);

  char orderStr[16] = ""; int o = 0;
  for (int k = 0; k < gRes.nCand; k++) o += snprintf(orderStr + o, sizeof(orderStr) - o, "%s\"%c\"", k ? "," : "", CFG[gRes.order[k]].id);
  int n = snprintf(buf, sizeof(buf),
    "{\"uptime_s\":%lu,\"system\":\"%s\",\"mode\":\"%s\",\"tank_ml\":%.0f,\"reserve_ml\":%.0f,"
    "\"available_after_ml\":%.0f,\"order\":[%s],\"reason\":\"%s\",\"plots\":[",
    up, sysName(gRes.sys), modeAuto ? "AUTO" : "MANUAL", gTankMl, PARAMS.reserveMl,
    gRes.availableAfter, orderStr, gReason);
  for (int i = 0; i < N_PLOTS; i++) {
    n += snprintf(buf + n, sizeof(buf) - n,
      "%s{\"id\":\"%c\",\"D\":%.1f,\"U\":%.0f,\"T\":%.1f,\"Q\":%.1f,\"state\":\"%s\",\"need_ml\":%.0f}",
      i ? "," : "", CFG[i].id, gRes.plot[i].D, gRes.plot[i].U, gRes.plot[i].T, gRes.plot[i].Q,
      stateName(gRes.plot[i].state), CFG[i].volumeMl);
  }
  snprintf(buf + n, sizeof(buf) - n, "]}");
  mqtt.publish(TOPIC_RECO, buf);
  publishPumpStatus("PERIODIC");
  Serial.println("[MQTT] data terkirim");
}

void onMqtt(char* topic, byte* payload, unsigned int len) {
  char cmd[24];
  unsigned int n = len < sizeof(cmd) - 1 ? len : sizeof(cmd) - 1;
  memcpy(cmd, payload, n); cmd[n] = 0;
  handleCommand(cmd, "MQTT");
}

void mqttMaintain() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqtt.connected()) {
    unsigned long now = millis();
    if (now - lastMqttTryMs < 5000UL) return;
    lastMqttTryMs = now;
    char cid[32]; snprintf(cid, sizeof(cid), "tanicerdas-esp32-%04lx", (unsigned long)(esp_random() & 0xFFFF));
    Serial.print("[MQTT] menghubungkan ke HiveMQ... ");
    if (mqtt.connect(cid, MQTT_USER, MQTT_PASS)) {
      Serial.println("terhubung");
      mqtt.subscribe(TOPIC_PUMP_CMD);
    } else {
      Serial.printf("gagal (rc=%d). Sistem tetap berjalan lokal.\n", mqtt.state());
    }
  }
  mqtt.loop();
}

// ================= PERINTAH =================
void handleCommand(char* cmd, const char* src) {
  // normalisasi: buang spasi di tepi + huruf besar
  while (*cmd == ' ' || *cmd == '\r' || *cmd == '\n') cmd++;
  int L = strlen(cmd);
  while (L > 0 && (cmd[L - 1] == ' ' || cmd[L - 1] == '\r' || cmd[L - 1] == '\n')) cmd[--L] = 0;
  for (int i = 0; i < L; i++) if (cmd[i] >= 'a' && cmd[i] <= 'z') cmd[i] -= 32;
  if (L == 0) return;
  Serial.printf("[CMD:%s] %s\n", src, cmd);

  if (!strcmp(cmd, "AUTO")) {
    if (estop) { Serial.println("  ditolak: emergency stop aktif"); return; }
    modeAuto = true; Serial.println("  mode AUTO aktif");
  } else if (!strcmp(cmd, "MANUAL")) {
    modeAuto = false; Serial.println("  mode MANUAL aktif");
  } else if (!strcmp(cmd, "STOP")) {
    modeAuto = false; stopPump("PERINTAH_STOP"); Serial.println("  pompa dihentikan, mode MANUAL");
  } else if (!strcmp(cmd, "PUMP_A") || !strcmp(cmd, "PUMP_B")) {
    int idx = (cmd[5] == 'A') ? 0 : 1;
    if (pumpOn)                            Serial.println("  ditolak: pompa sedang berjalan");
    else if (!pumpSafeNow())               Serial.println("  ditolak: kondisi tidak aman (sensor/tandon/ESTOP)");
    else if (gRes.plot[idx].state == ST_ERROR) Serial.println("  ditolak: sensor petak tidak valid");
    else if (CFG[idx].volumeMl > gTankMl - PARAMS.reserveMl) Serial.println("  ditolak: air tidak cukup untuk kebutuhan penuh");
    else startPump(idx, "MANUAL");
  } else {
    Serial.println("  perintah tidak dikenal (AUTO, MANUAL, STOP, PUMP_A, PUMP_B)");
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLen > 0) { serialBuf[serialLen] = 0; handleCommand(serialBuf, "SERIAL"); serialLen = 0; }
    } else if (serialLen < (int)sizeof(serialBuf) - 1) {
      serialBuf[serialLen++] = c;
    }
  }
}

// ================= TOMBOL & INDIKATOR =================
void buttonUpdate() {
  bool r = digitalRead(PIN_BTN_STOP);
  unsigned long now = millis();
  if (r != btnLast) { btnLast = r; btnChangeMs = now; }
  if (now - btnChangeMs > 50 && r != btnStable) {
    btnStable = r;
    if (btnStable == LOW) {               // tombol ditekan -> toggle emergency stop
      estop = !estop;
      if (estop) { modeAuto = false; stopPump("ESTOP"); }
      Serial.printf("[BTN] EMERGENCY STOP %s\n", estop ? "AKTIF (pompa dikunci)" : "DILEPAS");
    }
  }
}

void indicatorUpdate(unsigned long now) {
  bool blink = (now / 250UL) % 2UL == 0;
  for (int i = 0; i < N_PLOTS; i++) {
    int pin = i == 0 ? PIN_LED_A : PIN_LED_B;
    bool on;
    if (pumpOn && pumpTarget == i) on = blink;                 // berkedip = sedang disiram
    else on = (gRes.plot[i].state == ST_WATER);                // menyala = direkomendasikan
    digitalWrite(pin, on ? HIGH : LOW);
  }
  bool alert = (gRes.sys != SYS_OK) || estop;
  digitalWrite(PIN_LED_ALRT, alert ? HIGH : LOW);
  // Buzzer: bip 300 ms dengan sinyal ~500 Hz (aman untuk buzzer aktif maupun pasif)
  if (alert && ((now / 300UL) % 2UL == 0)) digitalWrite(PIN_BUZZER, (micros() / 1000UL) % 2UL);
  else digitalWrite(PIN_BUZZER, LOW);
}

// ================= SETUP & LOOP =================
void setup() {
  pinMode(PIN_RELAY, OUTPUT); setPump(false);           // keadaan aman dulu
  pinMode(PIN_LED_A, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_LED_ALRT, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== TaniCerdas - AirTepat | SIC Batch 8 ===");
  Serial.println("Perintah: AUTO | MANUAL | STOP | PUMP_A | PUMP_B");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.print("Menghubungkan WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) { delay(300); Serial.print("."); }
  Serial.println(WiFi.status() == WL_CONNECTED ? " terhubung" : " gagal (mode lokal)");

  netClient.setInsecure();      // simulasi/demo: TLS tanpa verifikasi sertifikat (lihat catatan di laporan)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(900);
  mqtt.setKeepAlive(30);

  lastWaterMs[0] = lastWaterMs[1] = millis();
  runCycle();
}

void loop() {
  unsigned long now = millis();
  handleSerial();
  buttonUpdate();
  mqttMaintain();
  pumpUpdate(now);
  if (now - lastCycleMs >= CYCLE_MS) { lastCycleMs = now; runCycle(); publishAll(false); }
  indicatorUpdate(millis());
}
