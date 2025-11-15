#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <MHZ19.h>

// --------------------- USER CONFIG ------------------------
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

const char* MQTT_HOST = "192.168.1.112";
const int   MQTT_PORT = 1883;
const char* MQTT_USER = "";
const char* MQTT_PASS = "";

const char* TOPIC_PM25  = "home/sensors/pm25";
const char* TOPIC_PM10  = "home/sensors/pm10";
const char* TOPIC_CO2   = "home/sensors/co2";
const char* TOPIC_RAD   = "home/sensors/radiation";
const char* TOPIC_TEMP  = "home/sensors/temp";
const char* TOPIC_HUM   = "home/sensors/hum";
const char* TOPIC_PRESS = "home/sensors/pressure";
// ----------------------------------------------------------

// SDS011
SoftwareSerial sdsSerial(16, 17); // RX, TX
// GDK101
#define GDK_PIN 34
// MH-Z14A
MHZ19 mhz19;
SoftwareSerial co2Serial(4, 5);  // RX, TX
// BME/BMP280
Adafruit_BME280 bme;

// WiFi + MQTT
WiFiClient espClient;
PubSubClient mqtt(espClient);

// -------------------- CONNECT WIFI ------------------------
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(500);
}

// --------------------- CONNECT MQTT -----------------------
void connectMQTT() {
    while (!mqtt.connected()) {
        mqtt.connect("esp32_sensors", MQTT_USER, MQTT_PASS);
        if (!mqtt.connected()) delay(1500);
    }
}

// --------------- READ SDS011 PARTICULATE SENSOR ----------
bool readSDS011(float &pm25, float &pm10) {
    uint8_t buf[10];

    if (sdsSerial.available() >= 10) {
        if (sdsSerial.read() == 0xAA && sdsSerial.peek() == 0xC0) {
            sdsSerial.readBytes(buf, 9);

            pm25 = (buf[2] * 256 + buf[1]) / 10.0;
            pm10 = (buf[4] * 256 + buf[3]) / 10.0;

            // checksum skip byte 9
            sdsSerial.read();
            return true;
        }
    }
    return false;
}

// -------------- READ GDK101 GEIGER COUNTER ----------------
// GDK101 outputs pulses → use pulseCount
volatile unsigned long pulseCount = 0;
void IRAM_ATTR countPulse() { pulseCount++; }

float readGDK101() {
    unsigned long counts = pulseCount;
    pulseCount = 0;
    return counts / 6.0; // convert CPM → μSv/h approx
}

// -------------------- READ MH-Z14A CO2 --------------------
int readCO2() {
    return mhz19.getCO2();
}

// ------------------ READ BME280 / BMP280 ------------------
void readBME(float &t, float &h, float &p) {
    t = bme.readTemperature();
    h = bme.readHumidity();
    p = bme.readPressure() / 100.0F; // hPa
}

// --------------------------- SETUP -------------------------
void setup() {
    Serial.begin(115200);

    // SDS011
    sdsSerial.begin(9600);

    // GDK101
    pinMode(GDK_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(GDK_PIN), countPulse, RISING);

    // MH-Z14A
    co2Serial.begin(9600);
    mhz19.begin(co2Serial);

    // BME/BMP280
    if (!bme.begin(0x76)) {
        Serial.println("BME/BMP280 NOT FOUND!");
    }

    connectWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
}

// --------------------------- LOOP -------------------------
void loop() {
    if (!mqtt.connected()) connectMQTT();
    mqtt.loop();

    float pm25, pm10;
    float temperature, humidity, pressure;
    int co2;
    float radiation;

    if (readSDS011(pm25, pm10)) {
        mqtt.publish(TOPIC_PM25, String(pm25).c_str(), true);
        mqtt.publish(TOPIC_PM10, String(pm10).c_str(), true);
    }

    co2 = readCO2();
    if (co2 > 0) mqtt.publish(TOPIC_CO2, String(co2).c_str(), true);

    readBME(temperature, humidity, pressure);
    mqtt.publish(TOPIC_TEMP, String(temperature).c_str(), true);
    mqtt.publish(TOPIC_HUM, String(humidity).c_str(), true);
    mqtt.publish(TOPIC_PRESS, String(pressure).c_str(), true);

    radiation = readGDK101();
    mqtt.publish(TOPIC_RAD, String(radiation).c_str(), true);

    delay(5000);
}
