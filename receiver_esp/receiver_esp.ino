#include "LoRa_E22.h"
#include "../shared/LoraData.h"
// LoRa_E22 e22(&Serial2, UART_BPS_RATE_9600);
#define M0_PIN 32
#define M1_PIN 33
#define AUX_PIN 25
#define LORA_PORT_TX 17
#define LORA_PORT_RX 16

LoRa_E22 e22(LORA_PORT_RX, LORA_PORT_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_115200);

FloatData floatData;

void print_config(LoRa_E22 *lora)
{  
  Serial.println(Serial2.baudRate());
  ResponseStructContainer response = lora->getConfiguration();
  Serial.println(Serial2.baudRate());
  Serial.println(response.status.getResponseDescription());
  if (response.status.code == E22_SUCCESS)
  {
    Configuration configuration = *(Configuration *)response.data;
    Serial.println("--------------------------------");
    Serial.print("Channel Description: ");
    Serial.println(configuration.getChannelDescription());
    Serial.print("ADDH: ");
    Serial.println(configuration.ADDH);
    Serial.print("ADDL: ");
    Serial.println(configuration.ADDL);
    Serial.print("CHAN: ");
    Serial.println(configuration.CHAN);
    Serial.print("Command: ");
    Serial.println(configuration.COMMAND);
    // Serial.println(configuration.CRYPT);
    Serial.print("Length: ");
    Serial.println(configuration.LENGHT);
    Serial.print("Net ID: ");
    Serial.println(configuration.NETID);
    // Serial.println(configuration.OPTION);
    Serial.print("STARTING_ADDRESS: ");
    Serial.println(configuration.STARTING_ADDRESS);
    Serial.print("TRANSMISSION_MODE: ");
    Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
    Serial.print("AirDataRate: ");
    Serial.println(configuration.SPED.getAirDataRateDescription());
    Serial.print("UARTBaudRate: ");
    Serial.println(configuration.SPED.getUARTBaudRateDescription());
  }
  else
  {
    Serial.println("Error!");
  }
}

void setup()
{
  Serial.begin(115200);                    // Bil seriportu
  // Serial2.begin(9600, SERIAL_8N1, 16, 17); // lora seriportu
  e22.begin();
  // print_config();
}

void loop()
{
  
  if(e22.available()){
    ResponseStructContainer rs = e22.receiveMessage(sizeof(FloatData));
    floatData = *(FloatData*)rs.data;
    Serial.println(rs.status.getResponseDescription());
    Serial.print("Team ID: "); Serial.print(floatData.teamID);
    Serial.print(" Counter: "); Serial.print(floatData.packageCounter);
    Serial.print(" Status: "); Serial.print(floatData.status);
    Serial.print(" Angle: "); Serial.print(floatData.tiltAngle);
    Serial.print(" Alt: "); Serial.print(floatData.altitude);
    Serial.print(" Lat: "); Serial.print(floatData.latitude, 6);
    Serial.print(" Long: "); Serial.print(floatData.longitude, 6);
    Serial.print(" GPS Alt: "); Serial.print(floatData.gpsAltitude);
    Serial.print(" g - Y: "); Serial.print(floatData.gY);
    Serial.print(" Angle Y: "); Serial.print(floatData.AngleY);
    Serial.print(" checkSum: "); Serial.println(floatData.checksum);
  }
  
  // Serial.println(Serial2.baudRate());
  // delay(1000);

  // delay(1000);

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
