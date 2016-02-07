/* 
 * Network Humidity Temperature Meter 
 * 
 * Consists of
 * Arduino Nano,
 * DHT22   - 1 unit,
 * DS18B20 - 3 units,
 * ENC28J60 based Ethernet controller.
 * Communicates using Modbus TCP.
 * 
 * (c) 2016, Mikhail Shiryaev
 */

// All the pins below are digital
#define DHT_PIN       5 // DHT22 data
#define ONE_WIRE_PIN0 2 // the 1st DS18B20 data
#define ONE_WIRE_PIN1 2 // the 2nd DS18B20 data
#define ONE_WIRE_PIN2 2 // the 3rd DS18B20 data

// MAC address
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
