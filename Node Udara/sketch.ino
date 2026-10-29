// NODE 1 - UDARA: DHT22 + LDR -> moving average -> fuzzy logic -> kipas + MQTT
#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

const char* BROKER = "broker.hivemq.com";
const char* TOPIC  = "greenhouse/udara/data";   // ganti prefix "greenhouse" jika bentrok
#define DHTPIN 15
#define LDRPIN 34
#define FANPIN 2

DHT dht(DHTPIN, DHT22);
WiFiClient wifi;
PubSubClient mqtt(wifi);

const int N = 5;                       // jendela moving average
float bt[N], bh[N], bl[N];
int idx = 0, cnt = 0;

float avg(float* b) { float s = 0; for (int i = 0; i < cnt; i++) s += b[i]; return s / cnt; }
float ramp(float x, float a, float b) { return x <= a ? 0 : x >= b ? 1 : (x - a) / (b - a); }

void connectAll() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin("Wokwi-GUEST", "", 6);
    while (WiFi.status() != WL_CONNECTED) delay(300);
  }
  while (!mqtt.connected()) {
    String id = "gh-udara-" + String(random(0xffff), HEX);
    if (!mqtt.connect(id.c_str())) delay(2000);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(FANPIN, OUTPUT);
  dht.begin();
  mqtt.setServer(BROKER, 1883);
}

void loop() {
  connectAll();
  mqtt.loop();

  float t = dht.readTemperature(), h = dht.readHumidity();
  if (isnan(t) || isnan(h)) { delay(2000); return; }
  float light = analogRead(LDRPIN) * 100.0 / 4095.0;

  // 1) Filtering: moving average
  bt[idx] = t; bh[idx] = h; bl[idx] = light;
  idx = (idx + 1) % N; if (cnt < N) cnt++;
  float T = avg(bt), H = avg(bh), L = avg(bl);

  // 2) Fuzzy logic: derajat keanggotaan 0..1
  float panas  = ramp(T, 28, 34);
  float lembap = ramp(H, 60, 80);
  float panasLembap = min(panas, lembap);          // aturan AND = min
  const char* status = "NYAMAN";
  if (panasLembap > 0.5) status = "PANAS-LEMBAP";
  else if (panas > 0.5)  status = "PANAS";
  bool fan = panas > 0.5;
  digitalWrite(FANPIN, fan);

  char msg[160];
  snprintf(msg, sizeof(msg),
    "{\"suhu\":%.1f,\"hum\":%.0f,\"cahaya\":%.0f,\"derajat_panas\":%.2f,\"status\":\"%s\",\"kipas\":\"%s\"}",
    T, H, L, panas, status, fan ? "ON" : "OFF");
  mqtt.publish(TOPIC, msg);
  Serial.println(msg);
  delay(2000);
}
