#ifndef PAYLOAD_PCB_HARDWARE_CONFIG_H
#define PAYLOAD_PCB_HARDWARE_CONFIG_H

//----LORA----
#define LORA_ADDH 0
#define LORA_ADDL 5
#define LORA_CHAN 21
#define LORA_M0_PIN 32
#define LORA_M1_PIN 33
#define LORA_AUX_PIN 25
#define LoraSerial Serial2
#define LORA_SERIAL_PORT_TX 17
#define LORA_SERIAL_PORT_RX 16

//----GPS----
#define GPSSerial Serial1
#define GPS_SERIAL_PORT_TX 14
#define GPS_SERIAL_PORT_RX 12

//----Debug Port----
#define DebugSerial Serial
#define DEBUG_SERIAL_PORT_TX 1
#define DEBUG_SERIAL_PORT_RX 3

//----Other----
#define I2C_SDA 21
#define I2C_SCL 22
#define BUZZER_PIN 13
#define ESC_PIN 2

#endif