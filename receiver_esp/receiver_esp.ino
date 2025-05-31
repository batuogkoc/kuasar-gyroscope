#include "LoRa_E22.h"
#include "../shared/LoraData.h"
#include "../shared/PayloadPCBHardwareConfig.h"

LoRa_E22 e22(LORA_SERIAL_PORT_RX, LORA_SERIAL_PORT_TX, &LoraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_115200);

FloatData receivedData;

void print_config(LoRa_E22 *lora)
{  
  Serial.println(LoraSerial.baudRate());
  ResponseStructContainer response = lora->getConfiguration();
  Serial.println(LoraSerial.baudRate());
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
  DebugSerial.begin(115200);
  e22.begin();
}

void loop()
{
  if (e22.available() > 0) {
    ResponseStructContainer rsc = e22.receiveMessage(sizeof(FloatData));
    if(rsc.status.code == E22_SUCCESS){
      memcpy(&receivedData, rsc.data, sizeof(FloatData));

      if (receivedData.header == LORA_DATA_HEADER) { //Headerı kontrol et bro
        DebugSerial.print("Team ID: "); DebugSerial.print(receivedData.teamID);
        DebugSerial.print(" Counter: "); DebugSerial.print(receivedData.packageCounter);
        DebugSerial.print(" Status: "); DebugSerial.print(receivedData.status);
        DebugSerial.print(" Angle: "); DebugSerial.print(receivedData.tiltAngle);
        DebugSerial.print(" Alt: "); DebugSerial.print(receivedData.altitude);
        DebugSerial.print(" Lat: "); DebugSerial.print(receivedData.latitude, 6);
        DebugSerial.print(" Long: "); DebugSerial.print(receivedData.longitude, 6);
        DebugSerial.print(" GPS Alt: "); DebugSerial.print(receivedData.gpsAltitude);
        DebugSerial.print(" g - Y: "); DebugSerial.print(receivedData.gY);
        DebugSerial.print(" Angle Y: "); DebugSerial.print(receivedData.AngleY);
        DebugSerial.print(" checkSum: "); DebugSerial.println(receivedData.checksum);
      }
      else{
        Serial1.println("Incorrect header!");
      }
    }
    rsc.close();
  }
}
