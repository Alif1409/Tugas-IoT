// NODE 2 - TANAH & AIR: potensiometer + HC-SR04 -> decision tree + regresi linear -> pompa + MQTT
// Penyiraman memperhatikan: ambang ganda (hysteresis), jendela jam siram (RTC DS1307), batas durasi, dan jeda antar siram.
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <RTClib.h>
#include <PubSubClient.h>

const char* BROKER = "broker.hivemq.com";
const char* T_DATA = "greenhouse/tanah/data";
const char* T_CTRL = "greenhouse/kontrol";
#define SOILPIN 34
#define TRIG 5
#define ECHO 18
#define PUMP 26

const float SOIL_MIN   = 30;     // % ambang kering: pompa mulai menyala di bawah ini
const float SOIL_OFF   = 45;     // % pompa berhenti setelah tanah mencapai ini (hysteresis)
const float SOIL_EMERGENCY = 15; // % sangat kering: boleh siram di luar jam siram

// Pengaturan waktu penyiraman
const bool DEMO_AWAL = false;    // keadaan awal mode demo. Bisa diubah saat berjalan lewat tombol dashboard (DEMO_ON / DEMO_OFF)
const int  WIN_START = 6, WIN_END = 10;           // jam siram otomatis (WIB): 06.00 sampai 09.59
// Sumber jam RTC (pilih salah satu):
//  1) SINKRON_NTP = true  : jam RTC diisi dari internet (jam dunia nyata, WIB) saat WiFi tersambung,
//                           lalu RTC menjaga waktu dan disinkron ulang tiap 6 jam. (DEFAULT)
//  2) ATUR_JAM_AWAL = true: jam RTC dipaksa ke AWAL_JAM:AWAL_MENIT saat boot (untuk menguji jendela siram)
const bool SINKRON_NTP   = true;
const bool ATUR_JAM_AWAL = false;
const int  AWAL_JAM = 5, AWAL_MENIT = 58;
const long GMT_OFFSET_S = 7 * 3600;                       // WIB = UTC+7
const unsigned long NTP_ULANG_MS = 6UL * 3600UL * 1000UL; // sinkron ulang tiap 6 jam
bool demoMode = DEMO_AWAL;       // true = abaikan jendela jam, durasi dan jeda dipendekkan
unsigned long pumpMaxMs()  { return demoMode ? 60000UL  : 30000UL;  }   // maks. lama pompa menyala per siklus
unsigned long pumpRestMs() { return demoMode ? 10000UL  : 300000UL; }   // jeda minimum antar siklus
const float TANK_DEPTH = 25;     // cm jarak sensor ke dasar tandon
const float LEVEL_MIN  = 5;      // cm minimal air
const unsigned long SAMPLE_MS = 1000;   // jeda antar sampel (ms)
const int   N = 5;

WiFiClient wifi;
PubSubClient mqtt(wifi);
float hist[N]; int cnt = 0;
String mode = "AUTO";            // AUTO | POMPA_ON | POMPA_OFF
bool pumpOn = false, hadRun = false;
unsigned long pumpStart = 0, pumpStop = 0;

RTC_DS1307 rtc;
bool rtcOk = false;

const char* sumberJam = "RTC";            // RTC (belum sinkron) | NTP | MANUAL
bool pernahSync = false;
unsigned long lastSync = 0, lastTry = 0;

// Isi RTC dari internet. Gagal -> coba lagi 30 detik kemudian, RTC tetap jalan dengan jam lamanya.
void sinkronNtp() {
  if (!SINKRON_NTP || ATUR_JAM_AWAL || !rtcOk || WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (lastTry != 0 && now - lastTry < 30000) return;
  if (pernahSync && now - lastSync < NTP_ULANG_MS) return;
  lastTry = now;
  configTime(GMT_OFFSET_S, 0, "pool.ntp.org", "time.nist.gov");
  struct tm t;
  if (getLocalTime(&t, 4000)) {
    rtc.adjust(DateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec));
    pernahSync = true; lastSync = now; sumberJam = "NTP";
  }
}

void onMsg(char* topic, byte* p, unsigned int len) {
  String m; for (unsigned int i = 0; i < len; i++) m += (char)p[i];
  m.trim();
  if (m == "AUTO" || m == "POMPA_ON" || m == "POMPA_OFF") mode = m;
  else if (m == "DEMO_ON")  demoMode = true;
  else if (m == "DEMO_OFF") demoMode = false;
}

void pub(const char* sub, const char* v) {      // publish nilai tunggal untuk widget app
  String t = String("greenhouse/tanah/") + sub;
  mqtt.publish(t.c_str(), v);
}

void connectAll() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("Wokwi-GUEST", "", 6);
    while (WiFi.status() != WL_CONNECTED) delay(300);
  }
  while (!mqtt.connected()) {
    String id = "gh-tanah-" + String(random(0xffff), HEX);
    if (mqtt.connect(id.c_str())) mqtt.subscribe(T_CTRL);
    else delay(2000);
  }
}

float readLevel() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);
  float dist = us * 0.0343 / 2;                 // cm sensor -> permukaan air
  return constrain(TANK_DEPTH - dist, 0, TANK_DEPTH);
}

// Regresi linear y = a + b*x pada N sampel terakhir; kembalikan menit sampai soil < SOIL_MIN
float predictMinutes() {
  if (cnt < N) return -1;
  float sx = 0, sy = 0, sxy = 0, sxx = 0;
  for (int i = 0; i < N; i++) { sx += i; sy += hist[i]; sxy += i * hist[i]; sxx += i * i; }
  float slope = (N * sxy - sx * sy) / (N * sxx - sx * sx);   // % per sampel (1 sampel = SAMPLE_MS)
  if (slope > -0.02) return -1;                              // tidak sedang mengering
  float last = hist[N - 1];
  if (last <= SOIL_MIN) return 0;
  return (last - SOIL_MIN) / (-slope) * SAMPLE_MS / 60000.0;
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT); pinMode(PUMP, OUTPUT);
  mqtt.setServer(BROKER, 1883);
  mqtt.setCallback(onMsg);
  Wire.begin();                                  // I2C: SDA = D21, SCL = D22
  rtcOk = rtc.begin();
  if (rtcOk) {
    if (ATUR_JAM_AWAL) { rtc.adjust(DateTime(2026, 10, 29, AWAL_JAM, AWAL_MENIT, 0)); sumberJam = "MANUAL"; }
    else if (!rtc.isrunning()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // agar RTC mulai berjalan
  }
}

void loop() {
  connectAll();
  sinkronNtp();
  unsigned long t0 = millis();
  while (millis() - t0 < SAMPLE_MS) { mqtt.loop(); delay(50); }   // tetap dengar perintah

  float soil = analogRead(SOILPIN) * 100.0 / 4095.0;
  float level = readLevel();
  if (cnt < N) hist[cnt++] = soil;
  else { for (int i = 0; i < N - 1; i++) hist[i] = hist[i + 1]; hist[N - 1] = soil; }

  // Jendela waktu: jika RTC tidak terdeteksi, jendela dianggap terbuka agar siram tidak terblokir
  DateTime wkt;
  bool haveTime = rtcOk;
  if (haveTime) wkt = rtc.now();
  int jam = haveTime ? wkt.hour() : 0, mnt = haveTime ? wkt.minute() : 0, det = haveTime ? wkt.second() : 0;
  bool windowOpen = demoMode || !haveTime || (jam >= WIN_START && jam < WIN_END);
  const char* jendela = demoMode ? "DEMO" : (!haveTime ? "JAM?" : (windowOpen ? "BUKA" : "TUTUP"));

  // Decision tree penyiraman
  bool pump = false;
  const char* alert = "-";
  bool tankOk = level > LEVEL_MIN;
  if (mode == "POMPA_OFF") {
    pump = false;
  } else if (mode == "POMPA_ON") {                 // manual: abaikan jam dan jeda, tetap cek tandon
    if (tankOk) pump = true; else alert = "TANDON HABIS";
  } else {                                         // AUTO
    bool emergency = soil < SOIL_EMERGENCY;
    bool want = pumpOn ? soil < SOIL_OFF : soil < SOIL_MIN;   // hysteresis: 30% nyala, 45% mati
    if (!want)                                      pump = false;
    else if (!tankOk)                               alert = "TANDON HABIS";
    else if (!windowOpen && !emergency)             alert = "DI LUAR JAM SIRAM";
    else if (pumpOn && millis() - pumpStart >= pumpMaxMs()) alert = "BATAS DURASI";
    else if (!pumpOn && hadRun && millis() - pumpStop < pumpRestMs()) alert = "JEDA SIRAM";
    else                                            pump = true;
  }
  if (pump && !pumpOn)  pumpStart = millis();
  if (!pump && pumpOn) { pumpStop = millis(); hadRun = true; }
  pumpOn = pump;
  digitalWrite(PUMP, pump);

  float pred = predictMinutes();
  char msg[380];
  snprintf(msg, sizeof(msg),
    "{\"soil\":%.0f,\"air_cm\":%.1f,\"pompa\":\"%s\",\"mode\":\"%s\",\"prediksi_kering_menit\":%.1f,\"alert\":\"%s\",\"jendela\":\"%s\",\"jam\":\"%02d:%02d:%02d\",\"tgl\":\"%04d-%02d-%02d\",\"sumber_jam\":\"%s\"}",
    soil, level, pump ? "ON" : "OFF", mode.c_str(), pred, alert, jendela, jam, mnt, det,
    haveTime ? wkt.year() : 0, haveTime ? wkt.month() : 0, haveTime ? wkt.day() : 0, sumberJam);
  mqtt.publish(T_DATA, msg);
  char b[16];
  snprintf(b, sizeof(b), "%.0f", soil); pub("soil", b);
  snprintf(b, sizeof(b), "%.1f", level); pub("air_cm", b);
  snprintf(b, sizeof(b), "%.1f", pred); pub("prediksi", b);
  pub("pompa", pump ? "ON" : "OFF");
  pub("mode", mode.c_str());
  pub("alert", alert);
  pub("jendela", jendela);
  pub("sumber_jam", sumberJam);
  Serial.println(msg);
}
