#include "LoRa_E22.h"
#include "../shared/LoraData.h"
#include "../shared/PayloadPCBHardwareConfig.h"

LoRa_E22 e22(LORA_SERIAL_PORT_RX, LORA_SERIAL_PORT_TX, &LoraSerial, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN, UART_BPS_RATE_115200);

FloatData floatData;

void initialize_dummy_data(FloatData* floatData){
  floatData->header = LORA_DATA_HEADER;
  floatData->teamID = LORA_DATA_TEAM_ID;
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

void setup()
{
  DebugSerial.begin(115200);                    // Bil seriportu
  e22.begin();
  initialize_dummy_data(&floatData);
}
void loop()
{
  DebugSerial.println(LoraSerial.baudRate());
  ResponseStatus resp = e22.sendFixedMessage(LORA_ADDH, LORA_ADDL, LORA_CHAN, &floatData, sizeof(floatData));
  DebugSerial.println(LoraSerial.baudRate());
  DebugSerial.println(resp.getResponseDescription());
  delay(1000);
}
