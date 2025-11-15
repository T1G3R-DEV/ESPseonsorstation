#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <Mhz19.h>

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
// SDS011 (Serial)
#define SDS_RX 16
#define SDS_TX 17
SoftwareSerial sdsSerial(SDS_RX, SDS_TX);

// MH-Z14A (CO2)
#define MHZ_RX 4
#define MHZ_TX 5
SoftwareSerial co2Serial(MHZ_RX, MHZ_TX);
Mhz19 co2Sensor;

// GDK101 (Radiation)
#define GDK_PIN 34
volatile unsigned long pulseCount = 0;
void IRAM_ATTR countPulse() { pulseCount++; }

// BME280
Adafruit_BME280 bme;

// -------------------- NETWORK ----------------------------
WiFiClient espClient;
PubSubClient mqtt(espClient);

// -------------------- CONNECT WIFI -----------------------
void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) delay(500);
}

// -------------------- CONNECT MQTT -----------------------
void connectMQTT() {
    while (!mqtt.connected()) {
        mqtt.connect("esp32_sensors", MQTT_USER, MQTT_PASS);
        if (!mqtt.connected()) delay(1500);
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
            return true;
        }
    }
    return false;
}

// -------------------- GDK101 READ ------------------------
float readGDK101() {
    unsigned long counts = pulseCount;
    pulseCount = 0;
    return counts / 6.0; // CPM → μSv/h approx
}

// -------------------- MH-Z14A READ -----------------------
int readCO2() {
    return co2Sensor.getCarbonDioxide(); // official API
}

// -------------------- BME280 READ -----------------------
void readBME(float &t, float &h, float &p) {
    t = bme.readTemperature();
    h = bme.readHumidity();
    p = bme.readPressure() / 100.0F; // hPa
}

// -------------------- SETUP -----------------------------
void setup() {
    Serial.begin(115200);

    // SDS011
    sdsSerial.begin(9600);

    // MH-Z14A
    co2Serial.begin(9600);
    co2Sensor.begin(&co2Serial);
    co2Sensor.setMeasuringRange(Mhz19MeasuringRange::Ppm_5000);
    co2Sensor.enableAutoBaseCalibration();

    Serial.println("Preheating CO2 sensor...");
    while (!co2Sensor.isReady()) delay(50);
    Serial.println("CO2 sensor ready.");

    // BME280
    if (!bme.begin(0x76)) {
        Serial.println("BME280 not found!");
    }

    // GDK101 interrupt
    pinMode(GDK_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(GDK_PIN), countPulse, RISING);

    // WiFi + MQTT
    connectWiFi();
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
}

// -------------------- LOOP ------------------------------
void loop() {
    if (!mqtt.connected()) connectMQTT();
    mqtt.loop();

    float pm25 = 0, pm10 = 0;
    float temperature = 0, humidity = 0, pressure = 0;
    int co2 = -1;
    float radiation = 0;

    // Read SDS011
    if (readSDS011(pm25, pm10)) {
        mqtt.publish(TOPIC_PM25, String(pm25).c_str(), true);
        mqtt.publish(TOPIC_PM10, String(pm10).c_str(), true);
    }

    // Read CO2
    co2 = readCO2();
    if (co2 > 0) mqtt.publish(TOPIC_CO2, String(co2).c_str(), true);

    // Read BME280
    readBME(temperature, humidity, pressure);
    mqtt.publish(TOPIC_TEMP,  String(temperature).c_str(), true);
    mqtt.publish(TOPIC_HUM,   String(humidity).c_str(), true);
    mqtt.publish(TOPIC_PRESS, String(pressure).c_str(), true);

    // Read GDK101
    radiation = readGDK101();
    mqtt.publish(TOPIC_RAD, String(radiation).c_str(), true);

    delay(5000); // 5 sec between loops
}
