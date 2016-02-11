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

#include <DHT.h>               // https://github.com/adafruit/DHT-sensor-library
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <UIPEthernet.h>       // https://github.com/ntruchsess/arduino_uip

// All the pins below are digital
#define DHT_PIN       2 // DHT22 data
#define ONE_WIRE_PIN0 5 // the 1st DS18B20 data
#define ONE_WIRE_PIN1 6 // the 2nd DS18B20 data
#define ONE_WIRE_PIN2 7 // the 3rd DS18B20 data

#define DHT_TYPE      DHT22 // DHT 22
#define ONE_WIRE_CNT  3     // count of 1-wire connections
#define TCP_PORT      502   // Modbus TCP

// DHT instance
DHT dht(DHT_PIN, DHT_TYPE);

// 1-wire and DS instances
OneWire oneWire0(ONE_WIRE_PIN0);
OneWire oneWire1(ONE_WIRE_PIN1);
OneWire oneWire2(ONE_WIRE_PIN2);
OneWire oneWires[] = {oneWire0, oneWire1, oneWire2};
DallasTemperature dsSensors[ONE_WIRE_CNT];

// Ethernet
byte mac[] = { 0xAB, 0xB6, 0xAA, 0x43, 0xA5, 0x9D };
IPAddress ip(192, 168, 1, 8);
EthernetServer server = EthernetServer(TCP_PORT);

// Speed control
unsigned long time;

// Initialize DHT sensor
void initDHT() {
  dht.begin();
  Serial.println("DHT sensor initialized");
}

// Initialize DS sensors
void initDS() {
  DeviceAddress deviceAddress;
  for (int i = 0; i < ONE_WIRE_CNT; i++) {
    dsSensors[i].setOneWire(&oneWires[i]);
    dsSensors[i].begin();
    if (dsSensors[i].getAddress(deviceAddress, 0)) 
      dsSensors[i].setResolution(deviceAddress, 12);
  }
  Serial.println("DS sensors initialized");
}

// Initialize ENC28J60 Ethernet controller
void initEthernet() {
  Ethernet.begin(mac, ip);
  server.begin();

  Serial.print("Ethernet initialized. IP: ");
  Serial.println(Ethernet.localIP());
}

void writeTime() {
  Serial.print("Time elapsed = ");
  Serial.println(millis() - time);
}

// Read data from DHT sensor
void readDHT() {
  writeTime();
  Serial.println("Reading from DHT sensor");
  float h = dht.readHumidity();
  float t = dht.readTemperature(); // *C
  Serial.println("Done");
  writeTime();

  if (isnan(h) || isnan(t)) {
    Serial.println("Error reading from DHT sensor");
  } else {
    Serial.print("h = ");
    Serial.print(h);
    Serial.println(" %");
    Serial.print("t = ");
    Serial.print(t);
    Serial.println(" *C");
  }
  writeTime();
  Serial.println();
}

// Read data from DS sensors
void readDS() {
  writeTime();
  Serial.println("Requesting DS sensors");
  for (int i = 0; i < ONE_WIRE_CNT; i++) {
    dsSensors[i].requestTemperatures();
  }
  Serial.println("Done");
  writeTime();
  
  for (int i=0; i < ONE_WIRE_CNT; i++) {
    float t = dsSensors[i].getTempCByIndex(0);
    Serial.print("t[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.print(t);
    Serial.println(" *C");
  }
  writeTime();
  Serial.println();
}

// Process Ethernet communication
void communicate() {
  size_t size;

  if (EthernetClient client = server.available()) {
    while((size = client.available()) > 0) {
      uint8_t* msg = (uint8_t*)malloc(size);
      size = client.read(msg, size);
      Serial.write(msg, size);
      free(msg);
    }

    client.println("DATA from Server!");
    client.stop();
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("NET TH meter started");
  initDHT();
  initDS();
  initEthernet();
  Serial.println();
}

void loop() {
  time = millis();
  
  // wait a few seconds between measurements
  delay(2000);

  // read temperature and humidity from DHT sensors, takes around 270 milliseconds
  readDHT();
  
  // read temperatures from DS sensors, takes around 2370 milliseconds
  readDS();

  // Ethernet communication
  communicate();
}
