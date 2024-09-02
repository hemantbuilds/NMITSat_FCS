#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <MPU6050.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define rxGPS 0  // RX pin for GPS
#define txGPS 1  // TX pin for GPS
#define SEALEVELPRESSURE_HPA (1013.25)
#define battery_percent 89
#define BUZZER_PIN 33
#define ESP32_CAM_PIN 34

File telemetryFile;
SoftwareSerial gpsSerial(rxGPS, txGPS); // GPS on software serial
TinyGPSPlus gps;
Adafruit_BME680 bme; // Create an instance for the BME680 sensor
MPU6050 mpu; // Create an instance for the MPU6050 sensor

void setup() {
    Serial.begin(9600);   // Connect serial to computer for debugging
    gpsSerial.begin(9600); // Connect GPS sensor to software serial
    Serial2.begin(115200);  // Connect XBee to Serial2
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(ESP32_CAM_PIN, OUTPUT);

      // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    //return;
  }
  Serial.println("SD card initialized.");
    // Open the telemetry file
  telemetryFile = SD.open("data.txt", FILE_WRITE);
  if (!telemetryFile) {
    Serial.println("Error opening telemetry file!");
    //return;
  }

    Wire.begin();
    if (!bme.begin()) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1);
    }
    mpu.initialize();
}
void loop(){
  telemetry();
  
  // Check for commands from XBee
  if (Serial2.available()) {
    String receivedCommand = Serial2.readString();

    if (receivedCommand.equalsIgnoreCase("a,b")) {
      digitalWrite(ESP32_CAM_PIN, HIGH);
      Serial.println("a,b");
       receive_note();
    } else if (receivedCommand.equalsIgnoreCase("c,d")) {
      digitalWrite(ESP32_CAM_PIN, LOW);
      Serial.println("c,d");
       receive_note();
    } else {
      Serial.println("Invalid command received");
    }

    // Provide audio feedback
    telem_note();
    
  }

}
void telem_note() {
  // Play a series of tones
  digitalWrite(BUZZER_PIN, HIGH);
  delay(350);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(120);
  digitalWrite(BUZZER_PIN, LOW);
}
void receive_note() {
  // Play a series of tones
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  // digitalWrite(BUZZER_PIN, LOW);
  // delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
  delay(50);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(150);
  digitalWrite(BUZZER_PIN, LOW);
}
void writeToSDCard(const String& data) {
 if (!telemetryFile)
      {
        Serial.println("Error opening file!");
      }
      else
      {
        // Write to the SD card
        
        telemetryFile.println(data);
        telemetryFile.close(); // Close the file after writing
        Serial.println("Data logged!!");
      }
}

void telemetry() {
    while (gpsSerial.available()) {   // Check for GPS data
        if (gps.encode(gpsSerial.read())) {   // Encode GPS data
            String dataToSend = "";
            // Collect MPU6050 data
            int16_t ax, ay, az, gx, gy, gz;
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            float accelX = ax / 16384.0;
            float accelY = ay / 16384.0;
            float accelZ = az / 16384.0;
            float gyroX = gx / 131.0;
            float gyroY = gy / 131.0;
            float gyroZ = gz / 131.0; 
            float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA) - 0; 
               

            // Collect GPS data
            dataToSend +=  String(accelX)+",";
            dataToSend +=  String(accelY)+",";
            dataToSend +=  String(accelZ)+",";
            dataToSend +=  String(bme.readAltitude(SEALEVELPRESSURE_HPA))+",";
            //dataToSend +=  String(altitude)+",";
            //dataToSend +=  String(gps.altitude.meters())+",";
            dataToSend +=  String(bme.readTemperature())+"," ;
            dataToSend +=  String(bme.readPressure() / 100.0F)+"," ;            
            dataToSend +=  String(gyroX) +",";
            dataToSend +=  String(gyroY)+"," ;
            dataToSend +=  String(gyroZ)+"," ; 
              // dataToSend +=  String("23.114186")+",";  
            // dataToSend +=  String("72.498518")+",";           
           dataToSend +=  String(gps.location.lat(), 6)+",";
           dataToSend +=  String(gps.location.lng(), 6)+",";
            dataToSend +=  String(gps.date.day())+ "/" +  String(gps.date.month()) +"/"+  String(gps.date.year())+",";
            dataToSend +=   String(battery_percent)+",";
            //dataToSend +=  String("0.109")+","; 
           dataToSend +=  String(gps.speed.mps())+",";
            dataToSend +=  String(gps.satellites.value())+",";
            //dataToSend +=  String(bme.readGas());
            dataToSend +=  String("humidity")+ "=" +  String((bme.humidity));
           // dataToSend += "Hour: " + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()) + "\n";

            // Collect BME680 data

            //dataToSend += "Altitude: " + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + " m\n";
            





             
            // Send data through XBee
            Serial2.println(dataToSend); 
            // Also print data to Serial Monitor for debugging
            Serial.println(dataToSend);  
                // Provide audio feedback
             telem_note();  
             //log the data in Sd card
             writeToSDCard(dataToSend);


            delay(1000); // Delay for stability
        }
    }
}