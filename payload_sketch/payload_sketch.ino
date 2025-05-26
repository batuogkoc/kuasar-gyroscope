#include <Wire.h>
// #include <BME280I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include "LoRa_E22.h"
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define buzzer 13

unsigned long lastSend = 0;
const unsigned long interval = 1000 / 6;
int counter = 0;
int prevAltitude = 0;
uint8_t paketSayaci = 0;
float baseAltitude = 0;
uint8_t status = 0;
float setpoint = 0;       // Başlangıçta hedef roll açısı (düz konum)
float roll_angle = 0;
int pwmSi = 1500;


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Servo esc;
Adafruit_BME280 bme;
TinyGPSPlus gps;
LoRa_E22 e22(&Serial2, UART_BPS_RATE_115200);

struct __attribute__((packed)) FloatData {
  uint8_t header = 115;
  uint8_t teamID = 132;
  uint8_t packageCounter = 0;
  uint8_t status = 0;
  uint8_t tiltAngle;
  uint16_t altitude;
  float latitude, longitude;
  float gpsAltitude;
  int16_t gY;
  uint8_t AngleY;
  uint8_t checksum = 0;
};

void setup() {
  pinMode(buzzer, OUTPUT);
  pinMode(2, OUTPUT);  //LED SANIRIM 
  digitalWrite(2, 1);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  digitalWrite(32, 0);
  digitalWrite(33, 0);
  Serial.println("I2C başlatıldı.");
  Wire.begin(21,22);
  Wire.setClock(400000);
  Serial1.begin(115200, SERIAL_8N1, 9, 10);
  Serial1.println("Serial Port Activated");
  Serial1.println("UART başlatıldı.");
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  Serial.begin(115200, SERIAL_8N1, 3, 1);
  bme.begin();
  bno.begin();
  e22.begin();

  delay(1000);

  bno.setExtCrystalUse(true);

  delay(200);
  esc.attach(2, 1000, 2000);      // ESC sinyal pini
  esc.writeMicroseconds(1500);    // motoru durdur
  delay(2000);  
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::SAMPLING_X2,
                  Adafruit_BME280::SAMPLING_NONE,
                  Adafruit_BME280::FILTER_X4);

  delay(1000);

  float altitudeSum = 0;
  const int AltitudeSamples = 100;

   for (int i = 0; i < AltitudeSamples; i++) {
    bme.takeForcedMeasurement();
    altitudeSum += bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(20);
  }

  baseAltitude = (altitudeSum / AltitudeSamples) - 40;
  Serial1.print("Base Altitude: ");
  Serial1.println(baseAltitude);
  delay(200);

}
FloatData readBNO055() {
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  float gY = gravity.y();
  float gMag = gravity.magnitude();
  FloatData d;
  d.tiltAngle = acos(gY / gMag) * RAD_TO_DEG;
  return d;
}

FloatData readBME280() {
  FloatData d;
  bme.takeForcedMeasurement();
  float rawAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  d.altitude = rawAltitude - baseAltitude;
  return d;
}

FloatData readGPS() {
  static FloatData lastValidGPS = {
    115,            // header
    132,            // takimID
    0,              // paketsayac
    0,              // durum
    0,            // tiltAngle
    0,            // altitude
    0.0, 0.0,       // latitude, longitude
    0.0,            // gpsAltitude
    0,            // gY
    0,            // AngleY
  };

  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }

  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastValidGPS.latitude = gps.location.lat();
    lastValidGPS.longitude = gps.location.lng();
  }

  if (gps.altitude.isValid() && gps.altitude.isUpdated()) {
    lastValidGPS.gpsAltitude = gps.altitude.meters();
  }

  return lastValidGPS;
}
uint16_t calculatePWM() {
  float setpoint = 0;       // Başlangıçta hedef roll açısı (düz konum)
  float roll_angle = 0;
  uint16_t pwmSignal;
  sensors_event_t orientation;
  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  roll_angle = orientation.orientation.x;

  if (roll_angle > 180) roll_angle -= 360;
  if (roll_angle < -180) roll_angle += 360;

  float error = setpoint - roll_angle;
  float K = 1.56;
  int correction = K * error;

  pwmSignal = 1500 + correction;
  pwmSignal = constrain(pwmSignal, 1000, 2000);
  esc.writeMicroseconds(pwmSignal);

  return pwmSignal;
}
 

void sendLoRa(FloatData ori, FloatData alt, FloatData gps) {
  FloatData d;
  d.header = 115;
  d.teamID = 132;
  d.packageCounter = paketSayaci;
  
  d.tiltAngle = ori.tiltAngle;
  d.altitude = constrain((uint16_t)alt.altitude, 0, 65535);
  d.latitude = gps.latitude;
  d.longitude = gps.longitude;
  d.gpsAltitude = gps.gpsAltitude;
 
   
  d.checksum = calculateChecksum(d);

  ResponseStatus rs = e22.sendFixedMessage(0x00, 0x02, 0x12, (uint8_t*)&d, sizeof(d));

  paketSayaci++;
  if (paketSayaci > 255) {
    paketSayaci = 0;
  }
}
 
uint8_t calculateChecksum(const FloatData& d) {
  const uint8_t* bytes = (const uint8_t*)&d;
  size_t length = sizeof(FloatData) - 1;  // checksum alanı hariç
  uint8_t sum = 0;

  for (size_t i = 0; i < length; i++) {
    sum ^= bytes[i];
  }

  return sum;
}
void loop() {
  if (millis() - lastSend >= interval) {
    lastSend = millis();
    FloatData ori = readBNO055();
    FloatData alt = readBME280();
    FloatData gps = readGPS();
    uint16_t pwmVal = calculatePWM();
   
    Serial1.print("Altitude: ");
    Serial1.print(alt.altitude);
    Serial1.print(" Tilt Angle: ");
    Serial1.print(ori.tiltAngle);
    Serial1.print(" Lat: ");
    Serial1.print(gps.latitude);
    Serial1.print(" Long: ");
    Serial1.print(gps.longitude);
    Serial1.print(" GPS Alt: ");
    Serial1.print(gps.gpsAltitude);
    Serial1.print(" | PWM: ");
    Serial1.println(pwmVal);

    
    Serial1.println();
    sendLoRa(ori, alt, gps);
  }
}
