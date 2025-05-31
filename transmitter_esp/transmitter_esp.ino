#include "LoRa_E22.h"
#define LORA_ADDH 0
#define LORA_ADDL 5
#define LORA_CHAN 21
#define M0_PIN 32
#define M1_PIN 33
#define AUX_PIN 25
#define LORA_PORT_TX 17
#define LORA_PORT_RX 16

LoRa_E22 e22(LORA_PORT_RX, LORA_PORT_TX, &Serial2, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_115200);

struct __attribute__((packed)) FloatData {
  uint8_t header;
  uint8_t teamID;
  uint8_t packageCounter = 0;
  uint8_t status;
  uint8_t tiltAngle;
  float altitude;
  float latitude, longitude;
  float gpsAltitude;
  int16_t gY;
  uint8_t AngleY;
  uint8_t checksum = 0;
};

FloatData floatData;

void initialize_data(FloatData* floatData){
  floatData->header = 115;
  floatData->teamID = 24;
  floatData->packageCounter = 0;
  floatData->status = 214;
  floatData->tiltAngle = 100;
  floatData->altitude = 123.2;
  floatData->latitude = 1284.3;
  floatData->longitude = 2882.4;
  floatData->gpsAltitude = 21742;
  floatData->gY = 2138;
  floatData->AngleY = 12;
  floatData->checksum = 0;
}

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
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // lora seriportu
  // pinMode(M0_PIN, OUTPUT);
  // pinMode(M1_PIN, OUTPUT);
  e22.begin();
  initialize_data(&floatData);
  // delay(500);
  // e22.begin();
}
void loop()
{
  Serial.println(Serial2.baudRate());
  ResponseStatus resp = e22.sendFixedMessage(LORA_ADDH, LORA_ADDL, LORA_CHAN, &floatData, sizeof(floatData));
  Serial.println(Serial2.baudRate());
  Serial.println(resp.getResponseDescription());
  delay(1000);

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
