/* 
 * Serial Humidity Temperature Meter 
 * 
 * Consists of
 * Arduino Nano,
 * DHT11/22 - 1 unit,
 * DS18B20  - 3 units,
 * 
 * Communicates using Modbus RTU:
 * Holding Registers (4x) 40001 - 40010
 * 
 * (c) 2017, Mikhail Shiryaev
 */

#include <DHT.h>               // https://github.com/adafruit/DHT-sensor-library
#include <OneWire.h>           // http://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library

// All the pins below are digital
#define DHT_PIN       2 // DHT22 data
#define ONE_WIRE_PIN0 5 // the 1st DS18B20 data
#define ONE_WIRE_PIN1 6 // the 2nd DS18B20 data
#define ONE_WIRE_PIN2 7 // the 3rd DS18B20 data

#define DHT_TYPE        DHT11  // DHT11 or DHT22
#define ONE_WIRE_CNT    3      // count of 1-wire connections
#define EMPTY_VAL       -100.0 // empty float value of measurement
#define MODBUS_HDR_LEN  9      // length of Modbus TCP header

#define DEBUG                  // write detailed log to Serial port

#ifdef DEBUG
 #define DEBUG_PRINT(val)       Serial.print(val)
 #define DEBUG_PRINTHEX(val)    Serial.print(val, HEX)
 #define DEBUG_PRINTLN(val)     Serial.println(val)
 #define DEBUG_WRITETIME()      writeTime()
#else
 #define DEBUG_PRINT(val)
 #define DEBUG_PRINTHEX(val)
 #define DEBUG_PRINTLN(val) 
 #define DEBUG_WRITETIME()
#endif

// DHT instance
DHT dht(DHT_PIN, DHT_TYPE);
float dhtH = EMPTY_VAL;
float dhtT = EMPTY_VAL;

// 1-wire and DS instances
OneWire oneWire0(ONE_WIRE_PIN0);
OneWire oneWire1(ONE_WIRE_PIN1);
OneWire oneWire2(ONE_WIRE_PIN2);
OneWire oneWires[] = {oneWire0, oneWire1, oneWire2};
DallasTemperature dsSensors[ONE_WIRE_CNT];
float dsT[ONE_WIRE_CNT] = { EMPTY_VAL, EMPTY_VAL, EMPTY_VAL };

// Modbus
const byte modbusHeader[MODBUS_HDR_LEN] = { 
  0x00, 0x00, // Transaction identifier
  0x00, 0x00, // Protocol identifier
  0x00, 0x17, // Length
  0x00,       // Unit identifier
  0x03,       // Function code
  0x14        // Byte count (20)
};

// Speed control
#ifdef DEBUG
unsigned long time;
#endif

// Initialize DHT sensor
void initDHT() {
  dht.begin();
  DEBUG_PRINTLN("DHT sensor initialized");
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
  DEBUG_PRINTLN("DS sensors initialized");
}

#ifdef DEBUG
void writeTime() {
  Serial.print("Time elapsed = ");
  Serial.println(millis() - time);
}
#endif

// Read data from DHT sensor
void readDHT() {
  DEBUG_WRITETIME();
  DEBUG_PRINTLN("Reading from DHT sensor");
  
  dhtH = dht.readHumidity();
  dhtT = dht.readTemperature(); // *C
  
  DEBUG_WRITETIME();

  if (isnan(dhtH) || isnan(dhtT)) {
    dhtH = EMPTY_VAL;
    dhtT = EMPTY_VAL;
    DEBUG_PRINTLN("Error reading from DHT sensor");
  } else {
    DEBUG_PRINT("h = ");
    DEBUG_PRINTLN(dhtH);
    DEBUG_PRINT("t = ");
    DEBUG_PRINTLN(dhtT);
  }
  
  DEBUG_WRITETIME();
  DEBUG_PRINTLN();
}

// Read data from DS sensor
void readDS(int index) {
  DEBUG_WRITETIME();
  DEBUG_PRINT("Requesting DS sensor ");
  DEBUG_PRINTLN(index);

  dsSensors[index].requestTemperatures();
  
  DEBUG_WRITETIME();
  
  dsT[index] = dsSensors[index].getTempCByIndex(0);

  DEBUG_PRINT("t = ");
  DEBUG_PRINTLN(dsT[index]);
  DEBUG_WRITETIME();
  DEBUG_PRINTLN();
}

// Process Ethernet communication
void communicate() {
  /*
  DEBUG_WRITETIME();

  // manage connection
  unsigned long inactiveSpan = millis() - commTime;
  
  if (clientConnected) {
    if (inactiveSpan > DISCONN_TIME) {
        DEBUG_PRINTLN("Disconnecting inactive client");
        client.stop();
        clientConnected = false;
    } else if (client.connected()) {
      DEBUG_PRINTLN("Client is still connected");
    } else {
      client.stop();
      clientConnected = false;
      DEBUG_PRINTLN("Client has been disconnected");
    }
  }

  if (!clientConnected) {    
    // connection is considered to be established only when data are received
    client = server.available();
    if (client) {
      clientConnected = true;
      DEBUG_PRINTLN("New client is connected");
    } else {
      DEBUG_PRINTLN("Client is not connected");
    }
  }

  // communicating
  if (clientConnected) {
    // wait for the request 00 00 00 00 00 06 01 03 00 00 00 0A
    DEBUG_PRINT("Receiving data: ");
    boolean sendResponse = false;
    while (client.available() > 0) {
      DEBUG_PRINT("_");
      char c = client.read();
      DEBUG_PRINTHEX(c);
  
      if (c == 0x0A) {
        sendResponse = true;
        commTime = millis();  
        inactiveSpan = 0;
        client.flush(); // clear input buffer
        break;
      }
    }
    DEBUG_PRINTLN();
  
    // send response
    if (sendResponse) {
      DEBUG_PRINTLN("Sending response");
      client.write(modbusHeader, MODBUS_HDR_LEN);
      client.write((byte*)&dhtH, 4);
      client.write((byte*)&dhtT, 4);
      client.write((byte*)&dsT, 12);
    } else {
      DEBUG_PRINTLN("Response not required");
    }
  }
  
#ifdef USE_WATCHDOG
  // reset Watchdog timer if client is active
  if (inactiveSpan <= REINIT_TIME) {
    wdt_reset();
  }
#else
  // reinit Ethernet if client can't connect
  if (inactiveSpan > REINIT_TIME) {
    initEthernet();
  }
#endif
  
  DEBUG_WRITETIME();
  DEBUG_PRINTLN();
  */
}

void setup() {
  Serial.begin(9600);
  Serial.println("SERIAL TH meter started");
  initDHT();
  initDS();
  DEBUG_PRINTLN();
}

void loop() {
#ifdef DEBUG
  time = millis();  
  delay(500);
#endif

  readDHT(); // around 270 ms
  communicate();
  readDS(0); // around 800 ms
  communicate();
  readDS(1); // around 800 ms
  communicate();
  readDS(2); // around 800 ms
  communicate();
}
