#include <Arduino.h>
#include <ModbusRTU.h>
#include "e100.h"
#include "ble_e100.h"
#include "wifi_e100.h"
#include "config_e100.h"
#include "PI4IOE5V6534Q.h"
#include <ADS7828.h>
#include "io_e100.h"
#include "eeprom.h"
#include "E100_control.h"
#include "modbus_e100.h"

#define   U65_IO_ADDR                   0x22    // For some reason address is multipled by 2 on the schematic
#define   EXP1_IO_ADDR                  0x48    // ADS7828 ADDRESS (SAME AS DATASHEET)           

#define   PORTB_12                      74
#define   PORTI_3                       72
#define   PORTD_3                       75
#define   PORTC_1                       73

//-----------------------------------------------//
//------------VFD addresses-------------------------//
//--------------------------------------------------//
#define   SpeedAdr              0x2001//0x091A
#define   RUNCOMAdr             0x2000//0x091B
#define   DirectionAdr          0x091C
#define   ExtFaultAdr           0x091D
#define   FaultResetAdr         0x091E
#define   StopMethod            0x0100
#define   DecelTime             0x010D // 0x0102
#define   VFD_SLAVE_ID                  1

ModbusRTU VFD;
int process_controller;

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.print("Request result: 0x");
  Serial.print(event, HEX);
  Serial.println("");
  return true;
}

void setup() {
  pinMode(D3, INPUT); // DDR Global RS-485 - Set to input so that the data direction on the 485 Txcvr is unspecified 
  pinMode(D4, INPUT); // DDR Local RS-485 - Set to input so that the data direction on the 485 Txcvr is unspecified 

  Serial.begin(9600);
  while(!Serial);

  Serial4.begin(9600, SERIAL_8N1);      // GLOBAL RS-485
  while(!Serial4);                      // Why is this Serial4 and not Serial6 as shown in the product description?

  init_config();
  Serial.println("Initialized!");
  #if defined(HARDWARE_GIGA)
    wifi_init();
    ble_init();
  #endif 

  io_setup();
  eeprom_setup();

  Serial2.begin(9600, SERIAL_8N1);     // Local RS-485
  VFD.begin(&Serial2, SERIAL_8N1);
  VFD.setBaudrate(9600);
  VFD.master();
  process_controller = 0; 
}

void loop() {
  // put your main code here, to run repeatedly:
  #if defined(HARDWARE_GIGA)
    ble_loop();
    wifi_loop();      // Process any WiFi related data
  #endif
  // checkADS7828();
  loop_two();   // This needs to remain here as it is an alternative loop for when BLE/WiFi connect 
}               // and become blocking functions

// This loop is called from WiFi/BLE when connected as they are blocking functions. Without this there will 
// be no main loop when they are connected. 
void loop_two()
{
  uint16_t mb_temp_value; 

  digital_io_loop();
  analog_io_loop();
  // If we are not under manual control, then run the automation routine!
  if (!test_config_parameter(CONFIG_PARAM_OP_STATE_ALL,OP_STATE_MANUAL_CONTROL)){
    E100control();
  }

  // // ***** Receive and process the global RS_485 data *****************
  // // Start by reading all available bytes in the buffer and storing them for processing
  // while (Serial4.available()){
  //   char thisChar = Serial4.read();
  //   modbuxRxData(thisChar, RS_485_BUFFER);
  // }

  // // Check the RS-485 Serial data MODBUS buffer and send any required responses to the Serial port
  // if (modbus_loop(RS_485_BUFFER)){
  //   int tx_bytes = get_tx_bytes(RS_485_BUFFER);
  //   Serial4.write(get_tx_buffer(RS_485_BUFFER), tx_bytes);
  // }

  // // *********  VFD Writes *****************
  // // 1 in 25 
  // if ((process_controller % 25) == 0){
  //   mb_temp_value = get_config_parameter(CONFIG_PARAM_VFD_ON_OFF_SETTING);
  //   VFD.writeHreg(VFD_SLAVE_ID, RUNCOMAdr, &mb_temp_value, 1, cbWrite);
  //   while (VFD.slave()) {
  //     VFD.task();
  //     yield();
  //   }
    
  //   mb_temp_value = get_config_parameter(CONFIG_PARAM_VFD_SPEED_SETTING);
  //   VFD.writeHreg(VFD_SLAVE_ID, SpeedAdr, &mb_temp_value, 1, cbWrite);
  //   while (VFD.slave()) {
  //     VFD.task();
  //     yield();
  //   }
  // }
  // process_controller++;
}

