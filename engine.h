// engine.h  --  Mesin keputusan TaniCerdas - AirTepat (tanpa dependensi Arduino)
// Rumus mengikuti proposal: Q = 0.50*D + 0.30*U + 0.20*T
#pragma once
#include <stdint.h>
#include <math.h>

#define N_PLOTS 2

enum PlotState : uint8_t {
  ST_ERROR = 0,      // sensor tidak valid
  ST_OK,             // cukup lembap, tidak perlu air
  ST_WATER,          // butuh air dan volume dialokasikan (SIRAM)
  ST_INSUFFICIENT,   // butuh air tetapi air tidak cukup untuk kebutuhan penuh
  ST_SOAK,           // baru disiram, menunggu waktu resapan
  ST_BLOCKED         // butuh air tetapi tandon tidak aman (rendah / sensor tandon error)
};

enum SystemState : uint8_t { SYS_OK = 0, SYS_SENSOR_ERROR, SYS_TANK_ERROR, SYS_TANK_LOW };

struct PlotConfig { char id; const char* crop; float target; float urgency; float volumeMl; };
struct PlotInput  { float moisture; bool valid; float minutesSince; bool inSoak; };
struct PlotResult { float D, U, T, Q; PlotState state; };
struct EngineParams { float tMaxMin; float reserveMl; };

struct EngineResult {
  PlotResult plot[N_PLOTS];
  int order[N_PLOTS];     // indeks petak kandidat, urut Q tertinggi -> terendah (-1 = kosong)
  int nCand;
  int firstWater;         // indeks petak pertama berstatus SIRAM (-1 = tidak ada)
  float availableAfter;   // sisa air terpakai-batas setelah alokasi (ml)
  SystemState sys;
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float calcD(float m, float target) { return target <= 0.0f ? 0.0f : clampf(100.0f * (target - m) / target, 0.0f, 100.0f); }
inline float calcT(float minutes, float tMax) { return tMax <= 0.0f ? 0.0f : clampf(100.0f * minutes / tMax, 0.0f, 100.0f); }
inline float calcQ(float D, float U, float T) { return 0.50f * D + 0.30f * U + 0.20f * T; }

inline EngineResult runEngine(const PlotConfig* cfg, const PlotInput* in,
                              float tankMl, bool tankValid, const EngineParams& p) {
  EngineResult r;
  r.nCand = 0; r.firstWater = -1; r.sys = SYS_OK; r.availableAfter = 0;
  for (int i = 0; i < N_PLOTS; i++) {
    r.order[i] = -1;
    r.plot[i].D = 0; r.plot[i].U = cfg[i].urgency; r.plot[i].T = 0; r.plot[i].Q = 0;
    r.plot[i].state = ST_OK;
  }
  // 1) Kualitas data: sensor tanah
  bool sensorsOk = true;
  for (int i = 0; i < N_PLOTS; i++) if (!in[i].valid) { r.plot[i].state = ST_ERROR; sensorsOk = false; }
  if (!sensorsOk) { r.sys = SYS_SENSOR_ERROR; return r; }
  // 2) Level air
  if (!tankValid) r.sys = SYS_TANK_ERROR;
  else if (tankMl < p.reserveMl) r.sys = SYS_TANK_LOW;
  // 3) Skor dan kandidat
  int cand[N_PLOTS]; int n = 0;
  for (int i = 0; i < N_PLOTS; i++) {
    r.plot[i].D = calcD(in[i].moisture, cfg[i].target);
    r.plot[i].T = calcT(in[i].minutesSince, p.tMaxMin);
    r.plot[i].Q = calcQ(r.plot[i].D, r.plot[i].U, r.plot[i].T);
    if (in[i].inSoak) r.plot[i].state = ST_SOAK;
    else if (r.plot[i].D > 0.0f) cand[n++] = i;
    else r.plot[i].state = ST_OK;
  }
  // 4) Urutkan Q tertinggi dulu (insertion sort)
  for (int a = 1; a < n; a++) {
    int key = cand[a]; int b = a - 1;
    while (b >= 0 && r.plot[cand[b]].Q < r.plot[key].Q) { cand[b + 1] = cand[b]; b--; }
    cand[b + 1] = key;
  }
  // 5) Alokasi anggaran air (tanpa penyiraman parsial)
  float avail = tankValid ? (tankMl - p.reserveMl) : 0.0f;
  if (avail < 0) avail = 0;
  for (int k = 0; k < n; k++) {
    int i = cand[k]; r.order[k] = i;
    if (r.sys != SYS_OK) r.plot[i].state = ST_BLOCKED;
    else if (cfg[i].volumeMl <= avail) {
      r.plot[i].state = ST_WATER; avail -= cfg[i].volumeMl;
      if (r.firstWater < 0) r.firstWater = i;
    } else r.plot[i].state = ST_INSUFFICIENT;
  }
  r.nCand = n; r.availableAfter = avail;
  return r;
}

inline const char* stateName(PlotState s) {
  switch (s) {
    case ST_ERROR: return "ERROR";
    case ST_OK: return "CUKUP";
    case ST_WATER: return "SIRAM";
    case ST_INSUFFICIENT: return "AIR_KURANG";
    case ST_SOAK: return "RESAP";
    default: return "TERBLOKIR";
  }
}
inline const char* sysName(SystemState s) {
  switch (s) {
    case SYS_OK: return "OK";
    case SYS_SENSOR_ERROR: return "SENSOR_ERROR";
    case SYS_TANK_ERROR: return "TANK_ERROR";
    default: return "TANK_LOW";
  }
}
