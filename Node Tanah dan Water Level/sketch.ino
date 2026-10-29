// NODE 2 - TANAH & AIR: potensiometer + HC-SR04 -> decision tree + regresi linear -> pompa + MQTT
#include <WiFi.h>
#include <PubSubClient.h>

const char* BROKER = "broker.hivemq.com";
const char* T_DATA = "greenhouse/tanah/data";
const char* T_CTRL = "greenhouse/kontrol";
#define SOILPIN 34
#define TRIG 5
#define ECHO 18
#define PUMP 26

const float SOIL_MIN   = 30;     // % ambang kering
const float TANK_DEPTH = 25;     // cm jarak sensor ke dasar tandon
const float LEVEL_MIN  = 5;      // cm minimal air
const unsigned long SAMPLE_MS = 1000;   // jeda antar sampel (ms)
const int   N = 5;

WiFiClient wifi;
PubSubClient mqtt(wifi);
float hist[N]; int cnt = 0;
String mode = "AUTO";            // AUTO | POMPA_ON | POMPA_OFF

void onMsg(char* topic, byte* p, unsigned int len) {
  String m; for (unsigned int i = 0; i < len; i++) m += (char)p[i];
  m.trim();
  if (m == "AUTO" || m == "POMPA_ON" || m == "POMPA_OFF") mode = m;
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
}

void loop() {
  connectAll();
  unsigned long t0 = millis();
  while (millis() - t0 < SAMPLE_MS) { mqtt.loop(); delay(50); }   // tetap dengar perintah

  float soil = analogRead(SOILPIN) * 100.0 / 4095.0;
  float level = readLevel();
  if (cnt < N) hist[cnt++] = soil;
  else { for (int i = 0; i < N - 1; i++) hist[i] = hist[i + 1]; hist[N - 1] = soil; }

  // Decision tree
  bool pump = false;
  const char* alert = "-";
  bool tankOk = level > LEVEL_MIN;
  if (mode == "POMPA_OFF") pump = false;
  else if (mode == "POMPA_ON" || soil < SOIL_MIN) {
    if (tankOk) pump = true;
    else alert = "TANDON HABIS";
  }
  digitalWrite(PUMP, pump);

  float pred = predictMinutes();
  char msg[200];
  snprintf(msg, sizeof(msg),
    "{\"soil\":%.0f,\"air_cm\":%.1f,\"pompa\":\"%s\",\"mode\":\"%s\",\"prediksi_kering_menit\":%.1f,\"alert\":\"%s\"}",
    soil, level, pump ? "ON" : "OFF", mode.c_str(), pred, alert);
  mqtt.publish(T_DATA, msg);
  char b[16];
  snprintf(b, sizeof(b), "%.0f", soil); pub("soil", b);
  snprintf(b, sizeof(b), "%.1f", level); pub("air_cm", b);
  snprintf(b, sizeof(b), "%.1f", pred); pub("prediksi", b);
  pub("pompa", pump ? "ON" : "OFF");
  pub("mode", mode.c_str());
  pub("alert", alert);
  Serial.println(msg);
}
