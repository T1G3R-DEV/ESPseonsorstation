#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <SensirionI2CScd4x.h>
#include <Wire.h>

// --------------------- USER CONFIG ------------------------
const char* WIFI_SSID = "USR8054";
const char* WIFI_PASS = "xxx";

const char* MQTT_HOST = "192.168.1.112";
const int   MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PASS = "mqtt_pass";

const char* TOPIC_PM25  = "home/sensors/pm25";
const char* TOPIC_PM10  = "home/sensors/pm10";
const char* TOPIC_CO2   = "home/sensors/co2";
const char* TOPIC_RAD   = "home/sensors/radiation";
const char* TOPIC_TEMP  = "home/sensors/temp";
const char* TOPIC_HUM   = "home/sensors/hum";
const char* TOPIC_PRESS = "home/sensors/pressure";
// ----------------------------------------------------------

// -------------------- SENSOR PINS ------------------------
#define SDS_RX 16
#define SDS_TX 17
SoftwareSerial sdsSerial(SDS_RX, SDS_TX);

Adafruit_BME280 bme;
#define GDK101_ADDRESS 0x18

// SCD41 CO₂ Sensor
SensirionI2CScd4x scd4x;

// -------------------- NETWORK ----------------------------
WiFiClient espClient;
PubSubClient mqtt(espClient);

// -------------------- CONNECT WIFI -----------------------
void connectWiFi() {
    Serial.println("[WiFi] Connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        attempt++;
        if (attempt > 40) { // 20 seconds timeout
            Serial.println("\n[WiFi] Failed to connect!");
            return;
        }
    }
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
}

// -------------------- CONNECT MQTT -----------------------
void connectMQTT() {
    Serial.println("[MQTT] Connecting...");
    while (!mqtt.connected()) {
        if (mqtt.connect("esp32_sensors", MQTT_USER, MQTT_PASS)) {
            Serial.println("[MQTT] Connected!");
        } else {
            Serial.print("[MQTT] Failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" Retrying in 1.5s");
            delay(1500);
        }
    }
}

// -------------------- SDS011 READ ------------------------
bool readSDS011(float &pm25, float &pm10) {
    uint8_t buf[10];
    if (sdsSerial.available() >= 10) {
        if (sdsSerial.read() == 0xAA && sdsSerial.peek() == 0xC0) {
            sdsSerial.readBytes(buf, 9);
            pm25 = (buf[2] * 256 + buf[1]) / 10.0;
            pm10 = (buf[4] * 256 + buf[3]) / 10.0;
            sdsSerial.read(); // drop checksum
            Serial.print("[SDS011] PM2.5: "); Serial.print(pm25);
            Serial.print(" µg/m³, PM10: "); Serial.println(pm10);
            return true;
        }
    }
    return false;
}

// -------------------- BME280 READ ------------------------
void readBME(float &t, float &h, float &p) {
    t = bme.readTemperature();
    h = bme.readHumidity();
    p = bme.readPressure() / 100.0F; // hPa
    Serial.print("[BME280] Temp: "); Serial.print(t);
    Serial.print(" °C, Hum: "); Serial.print(h);
    Serial.print(" %, Press: "); Serial.println(p);
}

// -------------------- GDK101 READ ------------------------
float readGDK101() {
    byte buffer[2];
    float radiation = -1.0;

    Wire.beginTransmission(GDK101_ADDRESS);
    Wire.write(0xB3); // 1-min avg
    if (Wire.endTransmission() != 0) {
        Serial.println("[GDK101] Transmission error");
        return -1;
    }

    delay(10);

    Wire.requestFrom(GDK101_ADDRESS, 2);
    if (Wire.available() == 2) {
        buffer[0] = Wire.read();
        buffer[1] = Wire.read();
        radiation = buffer[0] + buffer[1] / 100.0;
        Serial.print("[GDK101] Radiation: ");
        Serial.print(radiation);
        Serial.println(" uSv/hr");
    } else {
        Serial.println("[GDK101] No data received");
    }

    return radiation;
}

// -------------------- SETUP -----------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("[Setup] Starting...");

    // SDS011
    sdsSerial.begin(9600);
    Serial.println("[Setup] SDS011 initialized");

    // I2C (GDK101 + SCD41 + BME280)
    Wire.begin();

    // BME280
    if (!bme.begin(0x76)) {
        Serial.println("[Setup] BME280 not found!");
    } else {
        Serial.println("[Setup] BME280 initialized");
    }

    // SCD41 initialization
    uint16_t error;
    char errorMessage[256];

    scd4x.begin(Wire);
    scd4x.stopPeriodicMeasurement();

    error = scd4x.startPeriodicMeasurement();
    if (error) {
        scd4x.getErrorMessage(error, errorMessage, 256);
        Serial.print("[SCD41] Start error: ");
        Serial.println(errorMessage);
    } else {
        Serial.println("[Setup] SCD41 initialized");
    }

    Serial.println("[Setup] GDK101 I2C initialized");

    connectWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);

    Serial.println("[Setup] Setup complete");
}

// -------------------- LOOP ------------------------------
void loop() {
    if (!mqtt.connected()) {
        connectMQTT();
    }
    mqtt.loop();

    float pm25 = 0, pm10 = 0;
    float temperature = 0, humidity = 0, pressure = 0;
    float radiation = -1.0;

    // ---------------- SDS011 ----------------
    if (readSDS011(pm25, pm10)) {
        mqtt.publish(TOPIC_PM25, String(pm25).c_str(), true);
        mqtt.publish(TOPIC_PM10, String(pm10).c_str(), true);
    }

    // ---------------- SCD41 (CO₂ sensor) ----------------
    uint16_t co2_ppm;
    float scd_temp, scd_hum;
    uint16_t error;
    char errorMessage[256];

    error = scd4x.readMeasurement(co2_ppm, scd_temp, scd_hum);

    if (!error && co2_ppm != 0) {
        Serial.printf("[SCD41] CO2: %u ppm, Temp: %.2f °C, Hum: %.2f %%\n",
                      co2_ppm, scd_temp, scd_hum);

        mqtt.publish(TOPIC_CO2, String(co2_ppm).c_str(), true);
    }

    // ---------------- BME280 ----------------
    readBME(temperature, humidity, pressure);
    mqtt.publish(TOPIC_TEMP,  String(temperature).c_str(), true);
    mqtt.publish(TOPIC_HUM,   String(humidity).c_str(), true);
    mqtt.publish(TOPIC_PRESS, String(pressure).c_str(), true);

    // ---------------- GDK101 Radiation ----------------
    radiation = readGDK101();
    if (radiation >= 0) {
        mqtt.publish(TOPIC_RAD, String(radiation).c_str(), true);
    }

    delay(5000);
}
