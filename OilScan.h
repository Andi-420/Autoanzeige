/*
 * OilScan.h — Suche nach der herstellerspezifischen Oel-Temperatur (UDS Mode 22)
 *
 * Reine C-Logik ohne Arduino-Abhaengigkeiten (auf dem PC testbar):
 *   - uds_parse():   Antwort des ELM327 auf "22 xxxx" auswerten
 *   - oil_fml_eval(): Rohbytes mit einer der bekannten Temperatur-Formeln umrechnen
 *   - oil_scan_eval(): aus Kalt- und Warm-Werten die Oel-Kandidaten bestimmen
 *
 * Mode 22 ("ReadDataByIdentifier") liest nur – am Steuergeraet wird nichts veraendert.
 */
#pragma once
#include <stdint.h>
#include <string.h>

// ── Antwort auf "22 DDDD" auswerten ──────────────────────────────────
// Positive Antwort:  62 DD DD <Daten...>   (ggf. Multi-Frame "0: ... 1: ...")
// Negative Antwort:  7F 22 <NRC>           (0x31 = DID unbekannt)
// Rueckgabe: Anzahl Datenbytes (>= 0) bei positiver Antwort,
//            -1 bei negativer Antwort (nrc gesetzt), -2 bei keiner/kaputter Antwort.
#define UDS_NEG     (-1)
#define UDS_NONE    (-2)

static int uds_hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static int uds_parse(const char *resp, uint16_t did, uint8_t *out, int maxOut, uint8_t *nrc) {
  // Alle Zeilen zu einem Hex-Strom zusammensetzen; Multi-Frame-Praefixe "N:"
  // und die Laengenzeile ("00A") entfernen, Leerzeichen ignorieren.
  char hex[160]; int hl = 0;
  const char *p = resp;
  while (*p) {
    const char *eol = strchr(p, '\n');
    if (!eol) eol = p + strlen(p);
    char line[96]; int ll = 0;
    for (const char *q = p; q < eol && ll < 95; q++)
      if (*q != ' ' && *q != '\r') line[ll++] = *q;
    line[ll] = 0;
    p = (*eol) ? eol + 1 : eol;
    if (ll == 0) continue;

    const char *data = line;
    if (ll >= 2 && line[1] == ':' && uds_hexval(line[0]) >= 0) {
      data = line + 2;                               // "0:" / "1:" ...
    } else {
      bool allHex = true;
      for (int i = 0; i < ll; i++) if (uds_hexval(line[i]) < 0) { allHex = false; break; }
      if (!allHex) continue;                         // "SEARCHING...", "NO DATA", "?"
      if (ll <= 3) continue;                         // Laengenzeile "00A"
    }
    for (const char *d = data; *d && hl < (int)sizeof(hex) - 1; d++) hex[hl++] = *d;
  }
  hex[hl] = 0;

  char want[7];
  static const char *H = "0123456789ABCDEF";
  want[0] = '6'; want[1] = '2';
  want[2] = H[(did >> 12) & 0xF]; want[3] = H[(did >> 8) & 0xF];
  want[4] = H[(did >> 4) & 0xF];  want[5] = H[did & 0xF]; want[6] = 0;

  const char *pos = strstr(hex, want);
  if (pos) {
    pos += 6;
    int n = 0;
    while (pos[0] && pos[1] && n < maxOut) {
      int a = uds_hexval(pos[0]), b = uds_hexval(pos[1]);
      if (a < 0 || b < 0) break;
      out[n++] = (uint8_t)(a * 16 + b);
      pos += 2;
    }
    return n;
  }
  const char *neg = strstr(hex, "7F22");
  if (neg && uds_hexval(neg[4]) >= 0 && uds_hexval(neg[5]) >= 0) {
    if (nrc) *nrc = (uint8_t)(uds_hexval(neg[4]) * 16 + uds_hexval(neg[5]));
    return UDS_NEG;
  }
  return UDS_NONE;
}

// ── Temperatur-Formeln ──────────────────────────────────────────────
// Gaengige Kodierungen von Temperaturen in Steuergeraeten:
//   0: A - 40                 (wie die genormten OBD-PIDs)
//   1: A * 0.75 - 48          (aeltere VAG-Messwertbloecke)
//   2: (A*256+B) / 10 - 273.1 (Kelvin, 0.1er Aufloesung)
//   3: (int16)(A*256+B) / 10  (Grad C, 0.1er Aufloesung, mit Vorzeichen)
#define OIL_FML_COUNT 4
static const char *const OIL_FML_NAME[OIL_FML_COUNT] = { "A-40", "A*.75-48", "K/10", "C/10" };

static bool oil_fml_eval(int fml, const uint8_t *d, int len, float *out) {
  switch (fml) {
    case 0: if (len < 1 || len > 2) return false; *out = (float)d[0] - 40.0f; return true;
    case 1: if (len < 1 || len > 2) return false; *out = d[0] * 0.75f - 48.0f; return true;
    case 2: if (len != 2) return false; *out = ((d[0] << 8) | d[1]) / 10.0f - 273.1f; return true;
    case 3: if (len != 2) return false; *out = (int16_t)((d[0] << 8) | d[1]) / 10.0f; return true;
  }
  return false;
}

// ── Kandidaten bestimmen ────────────────────────────────────────────
struct OilHit {          // ein Treffer aus dem Kalt-Scan
  uint16_t did;
  uint8_t  len;          // Anzahl Datenbytes der Antwort
  uint8_t  cold[2];      // erste 2 Datenbytes, kalt
  uint8_t  warm[2];      // erste 2 Datenbytes, warm
  bool     warmOk;       // im Warm-Vergleich erneut gelesen
};

struct OilCand {
  uint16_t did;
  uint8_t  fml;
  float    cold, warm;
  bool     likeCoolant;  // verhaelt sich exakt wie Kuehlwasser -> eher nicht Oel
};

// Plausibel fuer Oel: kalt -30..70 C, warm 60..150 C und mindestens 20 K waermer.
// Pro DID wird nur die erste passende Formel genommen.
// Sortierung: zuerst Kandidaten, die NICHT wie Kuehlwasser aussehen.
static int oil_scan_eval(const OilHit *hits, int n, float coolCold, float coolWarm,
                         OilCand *out, int maxOut) {
  int cnt = 0;
  for (int i = 0; i < n && cnt < maxOut; i++) {
    const OilHit &h = hits[i];
    if (!h.warmOk) continue;
    for (int f = 0; f < OIL_FML_COUNT; f++) {
      float c, w;
      if (!oil_fml_eval(f, h.cold, h.len, &c) || !oil_fml_eval(f, h.warm, h.len, &w)) continue;
      if (c < -30 || c > 70 || w < 60 || w > 150 || w - c < 20) continue;
      OilCand &k = out[cnt++];
      k.did = h.did; k.fml = (uint8_t)f; k.cold = c; k.warm = w;
      float dc = c - coolCold, dw = w - coolWarm;
      k.likeCoolant = (dc > -2.5f && dc < 2.5f && dw > -2.5f && dw < 2.5f);
      break;
    }
  }
  // stabil sortieren: Nicht-Kuehlwasser zuerst
  for (int i = 1; i < cnt; i++) {
    OilCand k = out[i]; int j = i - 1;
    while (j >= 0 && out[j].likeCoolant && !k.likeCoolant) { out[j + 1] = out[j]; j--; }
    out[j + 1] = k;
  }
  return cnt;
}
