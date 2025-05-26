#include "Arduino.h"
// #include "HardwareSerial.h"
#include <Wire.h>
// #include <Adafruit_BNO055.h>
// #include <Adafruit_Sensor.h>


// === Nesneler ===
// Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(0xFFFFFFFF);
  Wire.setTimeout(0xFFFFFFFF);
  delay(2000);
  Serial.println("Timeouts");
  Serial.println(Wire.getTimeOut());
  Serial.println(Wire.getTimeout());


  //  if (!bno.begin()) {
  //    Serial.println("BNO055 not found");
  //    while (1);
  //  }
  //  Serial.println("BNO055 found");
}

uint8_t check_slave_connection(uint8_t address){
  Wire.beginTransmission(address);
  return Wire.endTransmission();
}


uint8_t scan_i2c(){
  uint8_t nDevices = 0;
  for(int address = 1; address < 127; address++ ) {
   uint8_t error = check_slave_connection(address)
   Serial.println(error);
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

void loop() { 
  uint8_t retval = check_slave_connection(40);
  Serial.println(retval);
  delay(200);
}
