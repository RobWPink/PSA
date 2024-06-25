// Bluetooth headers
#include <ArduinoBLE.h>

#include "e100.h"   // Required for loop_two()
#include "modbus_e100.h"

// JDY-33
//#define SERVICE_UUID              "0000ffe0-0000-1000-8000-00805f9b34fb"
//#define CHARACTERISTIC_UUID_RX    "0000ffe1-0000-1000-8000-00805f9b34fb"
//#define CHARACTERISTIC_UUID_TX    "0000ffe2-0000-1000-8000-00805f9b34fb"

#define SERVICE_UUID                "49535343-FE7D-4AE5-8FA9-9FAFD205E455"
#define CHARACTERISTIC_UUID_TX      "49535343-1E4D-4BD9-BA61-23C647249616"
#define CHARACTERISTIC_UUID_RX      "49535343-8841-43F4-A8D4-ECBE34729BB3"

BLEService modbusService(SERVICE_UUID); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLECharacteristic modbusCharacteristicRx(CHARACTERISTIC_UUID_RX, BLERead | BLEWrite, 16);
BLEByteCharacteristic modbusCharacteristicTx(CHARACTERISTIC_UUID_TX, BLERead | BLENotify);

unsigned long bleSocketDataTiming = 0;

void ble_init()
{
  // set advertised local name and service UUID:
  BLE.begin();
  BLE.setLocalName("E100");
  BLE.setAdvertisedService(modbusService);

  // add the characteristic to the service
  modbusService.addCharacteristic(modbusCharacteristicRx);
  modbusService.addCharacteristic(modbusCharacteristicTx);

  // add service
  BLE.addService(modbusService);

  // start advertising
  BLE.advertise();
}

void ble_loop()
{
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if device is conneced to our peripheral...
  if (central) {
    Serial.print("Connected to central: ");
    // Start the disconnection timeout.
    bleSocketDataTiming = millis();    
    // print the central's MAC address:
    Serial.println(central.address());

     // while the central is still connected to peripheral:
    while (central.connected()) {
      if (modbusCharacteristicRx.written()){     
        uint8_t small_buffer[16]; 
        uint8_t ble_rx_d = modbusCharacteristicRx.readValue(small_buffer, 16);
        Serial.print("BLE Byte RX bytes ");
        Serial.println(ble_rx_d);
        for(int i = 0; i < ble_rx_d; i++){
          modbuxRxData(small_buffer[i], BLE_BUFFER);
        }
      }

      if (modbus_loop(BLE_BUFFER)){
        int tx_bytes = get_tx_bytes(BLE_BUFFER);
        uint8_t * temp_ble_buffer_ptr = get_tx_buffer(BLE_BUFFER);
        Serial.print("BLE Byte TX bytes ");
        Serial.println(tx_bytes);
        for(int i = 0; i < tx_bytes; i++){
          modbusCharacteristicTx.writeValue(temp_ble_buffer_ptr[i]);
        }
        bleSocketDataTiming = millis();    // Reset the timer on connection, otherwise it will be immediatly disconnected if t>15s
      }else if ((millis() - bleSocketDataTiming) >= 15000){
        Serial.println();
        Serial.println("BLE Client TIMEOUT disconnected");
        central.disconnect();
      }
      loop_two();     // FUCKING Arduino!
    }
    Serial.println("Connected BLE disconnected");
  } // end if (central)
}

