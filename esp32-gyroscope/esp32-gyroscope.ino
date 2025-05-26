#include "Arduino.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>


// === Nesneler ===
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo esc;


// === Ayarlar ===
float setpoint = 0;       // Başlangıçta hedef roll açısı (düz konum)
float roll_angle = 0;
int pwm = 1500;

void setup() {
  int timeout=0;
  delay(1000);  
  Serial.begin(115200);
  Wire.begin();
  pinMode(19, OUTPUT);


  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1);
  }
  Serial.println("BNO055 found");

  bno.setExtCrystalUse(true);
  
  esc.attach(2, 1000, 2000);      // ESC sinyal pini
  esc.writeMicroseconds(1500);    // motoru durdur
  delay(2000);                    // ESC arming süresi
}

void loop() { 
  // Roll açısını al (x ekseni)
  sensors_event_t orientation;
  bno.getEvent(&orientation, Adafruit_BNO055::VECTOR_EULER);
  roll_angle = orientation.orientation.x;


  // Açıyı -180 ile +180 arasına düzelt
  if (roll_angle > 180) roll_angle -= 360;
  if (roll_angle < -180) roll_angle += 360;


  // Hata = setpoint - mevcut açı
  float error = setpoint - roll_angle;


  // Basit orantılı kontrol: PWM farkı = K * hata
  float K = 5.0;  // bu değeri sistemine göre ayarla
  int correction = K * error;


  // ESC PWM sinyali oluştur
  pwm = 1500 + correction;
  pwm = constrain(pwm, 1000, 2000);
  esc.writeMicroseconds(pwm);


  // Seri monitör
  Serial.print("Roll: ");
  Serial.print(roll_angle);
  Serial.print(" | PWM: ");
  Serial.println(pwm);


  // delay(20);  // ~50Hz kontrol döngüsü
}
