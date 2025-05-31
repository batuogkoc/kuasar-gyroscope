// #include "LoRa_E22.h"

HardwareSerial Serial11(PA10, PA9);
// HardwareSerial Serial22(PA3, PA2);

// LoRa_E22 e22(&Serial2, UART_BPS_RATE_115200);

// struct __attribute__((packed)) FloatData {
//   uint8_t header;
//   uint8_t teamID;
//   uint8_t packageCounter = 0;
//   uint8_t status;
//   uint8_t tiltAngle;
//   float altitude;
//   float latitude, longitude;
//   float gpsAltitude;
//   int16_t gY;
//   uint8_t AngleY;
//   uint8_t checksum = 0;
// };

// FloatData receivedData;

void setup() {
  Serial11.begin(115200); // Bil seriportu
  // Serial22.begin(115200); // lora seriportu
  // delay(500);
  // e22.begin();
  Serial11.println("Starting up");
}

void loop() {
  Serial11.println("Starting up");
  delay(1000);
  // if (e22.available() > 0) {
  //   ResponseStructContainer rsc = e22.receiveMessage(sizeof(FloatData));
  //   memcpy(&receivedData, rsc.data, sizeof(FloatData));
  //   rsc.close();

  //   if (receivedData.header == 115) { //Headerı kontrol et bro
  //     Serial11.print("Team ID: "); Serial11.print(receivedData.teamID);
  //     Serial11.print(" Counter: "); Serial11.print(receivedData.packageCounter);
  //     Serial11.print(" Status: "); Serial11.print(receivedData.status);
  //     Serial11.print(" Angle: "); Serial11.print(receivedData.tiltAngle);
  //     Serial11.print(" Alt: "); Serial11.print(receivedData.altitude);
  //     Serial11.print(" Lat: "); Serial11.print(receivedData.latitude, 6);
  //     Serial11.print(" Long: "); Serial11.print(receivedData.longitude, 6);
  //     Serial11.print(" GPS Alt: "); Serial11.print(receivedData.gpsAltitude);
  //     Serial11.print(" g - Y: "); Serial11.print(receivedData.gY);
  //     Serial11.print(" Angle Y: "); Serial11.print(receivedData.AngleY);
  //     Serial11.print(" checkSum: "); Serial11.println(receivedData.checksum);
  //   }
  //   else{
  //     Serial1.println("Incorrect header!");
  //   }
  // }
}
