#include "Arduino.h"
// #include "HardwareSerial.h"
#include <Wire.h>
// #include "soc/soc_caps.h"
// #include "esp32-hal-i2c.h"
extern "C" {
  #include "driver/i2c.h"
  #include "driver/gpio.h"
  #include "esp_system.h"
}
// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>


// === Nesneler ===
// Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  delay(1000);  
  Serial.begin(115200);
  // Wire.begin(16, 17, 400000);
  Wire.begin();
  int timeout=0;
  // i2c_get_timeout(I2C_NUM_0, &timeout);
  Serial.print("Old timeout: ");
  Serial.println(timeout);
  // i2c_set_timeout(I2C_NUM_0, 16000);
  

  i2c_get_timeout(I2C_NUM_0, &timeout);
  Serial.print("New timeout: ");
  Serial.println(timeout);
  // Wire.setTimeOut(0xFFFFFFFF);
  // Wire.setTimeout(0xFFFFFFFF);
  // Wire.setTimeOut(0);
  // Wire.setTimeout(0);

  Serial.println("Timeouts");
    // Serial.println(Wire.getTimeOut());
  // Serial.println(Wire.getTimeout());
  pinMode(19, OUTPUT);


  //  if (!bno.begin()) {
  //    Serial.println("BNO055 not found");
  //    while (1);
  //  }
  //  Serial.println("BNO055 found");
}

uint8_t check_slave_connection(uint8_t address){
  Wire.beginTransmission(address);
  // Wire.write(0);
  return Wire.endTransmission();
}


uint8_t scan_i2c(){
  uint8_t nDevices = 0;
  for(int address = 1; address < 127; address++ ) {
   uint8_t error = check_slave_connection(address);
  //  Serial.println(error);
   if (error == 0) {
     Serial.print("I2C device found at address 0x");
     if (address<16) {
       Serial.print("0");
     }
     Serial.println(address,HEX);
     nDevices++;
   }
   else if (error==4) {
     Serial.print("Unknow error at address 0x");
     if (address<16) {
       Serial.print("0");
     }
     Serial.println(address,HEX);
   }   
   delay(1); 
 }
 return nDevices;
}

void search_loop(){
   Serial.println("Scanning...");
 
   int nDevices = scan_i2c();
   if (nDevices == 0) {
     Serial.println("No I2C devices found\n");
   }
   else {
     Serial.println("done\n");
   }
   delay(1000);    
}

void bno_test_loop(){
  digitalWrite(19, HIGH);
  uint8_t retval = check_slave_connection(40);
  // delay(200);
  digitalWrite(19, LOW);
  Serial.println(retval);
  delay(200);
}

void loop() { 
  // search_loop();
  check_slave_connection(0x77);
  check_slave_connection(40);
  delay(100);
}
