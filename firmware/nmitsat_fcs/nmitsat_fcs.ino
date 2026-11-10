/**
 * NMITSat_FCS — CanSat Flight Control Software
 *
 * Target board: Teensy 4.1 (OBC)
 * Language: C++ (Arduino IDE / Teensyduino)
 *
 * Responsibilities:
 *  - Sample GPS, BME680, MPU6050
 *  - Stream CSV telemetry to Ground Station via XBee (Serial2)
 *  - Log the same line to the onboard SD card
 *  - GCS commands for ESP32-CAM enable / disable + buzzer cues
 *
 * See docs/wiring.md and docs/telemetry.md
 */

#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <MPU6050.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

// -------------------- Pins / radios --------------------
// GPS on Hardware Serial1 (Teensy 4.1: RX1=0, TX1=1)
#define GPS_BAUD 9600
#define XBEE_BAUD 115200
#define DEBUG_BAUD 9600

#define SEALEVELPRESSURE_HPA (1013.25f)
#define BATTERY_PERCENT_PLACEHOLDER 89  // replace with ADC read when wired

#define BUZZER_PIN 33
#define ESP32_CAM_PIN 34
#define SD_CS BUILTIN_SDCARD

#define TELEMETRY_FILENAME "data.txt"

// Planned recovery altitudes from mission notes (not yet automated in this sketch)
const float APOGEE_TRIGGER_M = 600.0f;
const float CHUTE_DEPLOY_M = 500.0f;

TinyGPSPlus gps;
Adafruit_BME680 bme;
MPU6050 mpu;

File telemetryFile;
bool sdReady = false;

void telem_note();
void receive_note();
bool openTelemetryFile();
void writeToSDCard(const String& data);
void telemetry();
void handleGroundCommands();

void setup() {
  Serial.begin(DEBUG_BAUD);     // USB debug
  Serial1.begin(GPS_BAUD);      // GPS
  Serial2.begin(XBEE_BAUD);     // XBee ↔ GCS

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(ESP32_CAM_PIN, OUTPUT);
  digitalWrite(ESP32_CAM_PIN, LOW);

  // SD card (Teensy 4.1 built-in slot)
  if (!SD.begin(SD_CS)) {
    Serial.println(F("SD card initialization failed!"));
    sdReady = false;
  } else {
    Serial.println(F("SD card initialized."));
    sdReady = openTelemetryFile();
  }

  Wire.begin();

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1) {
      telem_note();
      delay(500);
    }
  }

  // Oversampling / filter recommended by Adafruit BME680 examples
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320°C for 150 ms

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 not responding — check I2C wiring"));
  }

  Serial.println(F("NMITSat_FCS ready"));
  telem_note();
}

void loop() {
  telemetry();
  handleGroundCommands();
}

void handleGroundCommands() {
  if (!Serial2.available()) {
    return;
  }

  String receivedCommand = Serial2.readStringUntil('\n');
  receivedCommand.trim();

  if (receivedCommand.equalsIgnoreCase("a,b")) {
    // Camera ON
    digitalWrite(ESP32_CAM_PIN, HIGH);
    Serial.println(F("CMD camera ON (a,b)"));
    receive_note();
  } else if (receivedCommand.equalsIgnoreCase("c,d")) {
    // Camera OFF
    digitalWrite(ESP32_CAM_PIN, LOW);
    Serial.println(F("CMD camera OFF (c,d)"));
    receive_note();
  } else {
    Serial.print(F("Invalid command: "));
    Serial.println(receivedCommand);
  }

  telem_note();
}

void telem_note() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(350);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(120);
  digitalWrite(BUZZER_PIN, LOW);
}

void receive_note() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
}

bool openTelemetryFile() {
  telemetryFile = SD.open(TELEMETRY_FILENAME, FILE_WRITE);
  if (!telemetryFile) {
    Serial.println(F("Error opening telemetry file!"));
    return false;
  }
  return true;
}

void writeToSDCard(const String& data) {
  if (!sdReady) {
    return;
  }

  // Re-open if a previous close left the handle invalid
  if (!telemetryFile) {
    if (!openTelemetryFile()) {
      return;
    }
  }

  telemetryFile.println(data);
  telemetryFile.flush();
  Serial.println(F("Data logged to SD"));
}

void telemetry() {
  while (Serial1.available() > 0) {
    if (!gps.encode(Serial1.read())) {
      continue;
    }

    // Prefer a fresh BME reading once per packet
    if (!bme.performReading()) {
      Serial.println(F("BME680 reading failed"));
      continue;
    }

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float accelX = ax / 16384.0f;
    float accelY = ay / 16384.0f;
    float accelZ = az / 16384.0f;
    float gyroX = gx / 131.0f;
    float gyroY = gy / 131.0f;
    float gyroZ = gz / 131.0f;

    // Barometric altitude from last BME680 reading
    float altitude_m = bme.readAltitude(SEALEVELPRESSURE_HPA);

    // Mission notes: second chute when descending through ~500 m after ~600 m apogee.
    // Hook recovery actuators here when hardware is wired.
    (void)APOGEE_TRIGGER_M;
    (void)CHUTE_DEPLOY_M;

    String dataToSend = "";
    dataToSend += String(accelX) + ",";
    dataToSend += String(accelY) + ",";
    dataToSend += String(accelZ) + ",";
    dataToSend += String(altitude_m) + ",";
    dataToSend += String(bme.temperature) + ",";
    dataToSend += String(bme.pressure / 100.0f) + ",";
    dataToSend += String(gyroX) + ",";
    dataToSend += String(gyroY) + ",";
    dataToSend += String(gyroZ) + ",";
    dataToSend += String(gps.location.lat(), 6) + ",";
    dataToSend += String(gps.location.lng(), 6) + ",";
    dataToSend += String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year()) + ",";
    dataToSend += String(BATTERY_PERCENT_PLACEHOLDER) + ",";
    dataToSend += String(gps.speed.mps()) + ",";
    dataToSend += String(gps.satellites.value()) + ",";
    dataToSend += "humidity=" + String(bme.humidity);

    // Downlink + local debug
    Serial2.println(dataToSend);
    Serial.println(dataToSend);

    telem_note();
    writeToSDCard(dataToSend);

    delay(1000);
  }
}
