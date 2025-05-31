#include <Wire.h>
// #include <BME280I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include "LoRa_E22.h"
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include "../shared/LoraData.h"
#include "../shared/PayloadPCBHardwareConfig.h"

#define SEALEVELPRESSURE_HPA (1013.25)

LoRa_E22 e22(LORA_SERIAL_PORT_RX, LORA_SERIAL_PORT_TX, &LoraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_115200);

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

  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
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
  d.header = LORA_DATA_HEADER;
  d.teamID = LORA_DATA_TEAM_ID;
  d.packageCounter = paketSayaci;
  
  d.tiltAngle = ori.tiltAngle;
  d.altitude = constrain((uint16_t)alt.altitude, 0, 65535);
  d.latitude = gps.latitude;
  d.longitude = gps.longitude;
  d.gpsAltitude = gps.gpsAltitude;
 
   
  d.checksum = calculateChecksum(d);

  ResponseStatus rs = e22.sendFixedMessage(LORA_ADDH, LORA_ADDL, LORA_CHAN, &d, sizeof(d));
  DebugSerial.println(rs.getResponseDescription());

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


void setup() {
  DebugSerial.begin(115200, SERIAL_8N1, 3, 1); //serial debug
  DebugSerial.println("Serial (Debug) Activated");
  GPSSerial.begin(115200, SERIAL_8N1, GPS_SERIAL_PORT_RX, GPS_SERIAL_PORT_TX); //gps
  DebugSerial.println("Serial1 (GPS) Activated");
  // Serial2.begin(115200, SERIAL_8N1, 16, 17); //lora
  // DebugSerial.println("Serial2 Activated");

  pinMode(BUZZER_PIN, OUTPUT);
  // pinMode(2, OUTPUT);  //LED SANIRIM 
  // digitalWrite(2, 1);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  DebugSerial.println("I2C Activated");
  bme.begin();
  bno.begin();
  e22.begin();

  delay(1000);

  bno.setExtCrystalUse(true);

  delay(200);
  esc.attach(ESC_PIN, 1000, 2000);      // ESC sinyal pini
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
  DebugSerial.print("Base Altitude: ");
  DebugSerial.println(baseAltitude);
  delay(200);

  DebugSerial.println("Setup Complete");
}

void loop() {
  if (millis() - lastSend >= interval) {
    lastSend = millis();
    FloatData ori = readBNO055();
    FloatData alt = readBME280();
    FloatData gps = readGPS();
    uint16_t pwmVal = calculatePWM();
   
    DebugSerial.print("Altitude: ");
    DebugSerial.print(alt.altitude);
    DebugSerial.print(" Tilt Angle: ");
    DebugSerial.print(ori.tiltAngle);
    DebugSerial.print(" Lat: ");
    DebugSerial.print(gps.latitude);
    DebugSerial.print(" Long: ");
    DebugSerial.print(gps.longitude);
    DebugSerial.print(" GPS Alt: ");
    DebugSerial.print(gps.gpsAltitude);
    DebugSerial.print(" | PWM: ");
    DebugSerial.println(pwmVal);

    
    DebugSerial.println();
    sendLoRa(ori, alt, gps);
  }
}
