#include <WiFi.h>
#include "modbus_e100.h"
#include "config_e100.h"

#define TCP_SERIAL_BUFFER_LIMIT       256
#define PACKET_CHECK_ONLY             1
#define PACKET_CRC_INSERT             0
uint8_t TCPSerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t TCPSerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t RSSerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t RSSerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t BLESerialBufferRx[TCP_SERIAL_BUFFER_LIMIT];
uint8_t BLESerialBufferTx[TCP_SERIAL_BUFFER_LIMIT];
int TCPSerialBufferPt = 0; 
int RSSerialBufferPt = 0; 
int BLESerialBufferPt = 0; 
int tcp_tx_bytes = 0;
int rs_tx_bytes = 0;
int ble_tx_bytes = 0;

bool CRC16(uint8_t * packet, int dataLength, char checkElseInsert) //CRC 16 for modbus checksum
{
    unsigned int CheckSum;
    unsigned int j;
    unsigned char lowCRC, highCRC;
    unsigned short i;

    CheckSum = 0xFFFF;
    for (j = 0; j < dataLength; j++)
    {
        CheckSum = CheckSum ^ (unsigned int)packet[j];
        for(i = 8; i > 0; i--)
            if((CheckSum) & 0x0001)
                CheckSum = (CheckSum >> 1) ^ 0xa001;
            else
                CheckSum >>= 1;
    }
    highCRC = (CheckSum >> 8) & 0xFF;
    lowCRC = (CheckSum & 0xFF);

    if (checkElseInsert == 1){
        if ((packet[dataLength+1] == highCRC) & (packet[dataLength] == lowCRC ))
            return 1;
        else
            return 0;
    }else{
        packet[dataLength] = lowCRC;
        packet[dataLength+1] = highCRC;
        return 1;
    }  
}

uint16_t readRegResponse(int i){
  uint16_t reg_response_val = 0;

  switch(i){
    case MODBUS_REG_RELAYS:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_RELAYS, MSB);
      break;
    case MODBUS_REG_RELAYS_2:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_RELAYS, LSB);
      break;
    case MODBUS_REG_INPUTS:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_INPUTS, MSB);
      Serial.print("Inputs MB: ");
      Serial.println(reg_response_val);
      break;
    case MODBUS_REG_EVO_GAS_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_EVO_GAS_PX, AS_INT);
      break;
    case MODBUS_REG_PSA_PRESS_1_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_PSA_PRESS_1_PX, AS_INT);
      break;
    case MODBUS_REG_PSA_PRESS_2_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_PSA_PRESS_2_PX, AS_INT);
      break;
    case MODBUS_REG_FINAL_GAS_DISCHARGE_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_FINAL_GAS_DISCHARGE_PX, AS_INT);
      break;
    case MODBUS_REG_ACT_1_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_ACT_1_PX, AS_INT);
      break;
    case MODBUS_REG_ACT_2_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_ACT_2_PX, AS_INT);
      break;
    case MODBUS_REG_PSA_1_PX:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_PSA_1_PX, AS_INT);
      break;
    case MODBUS_REG_GAS_SUCTION_PX:
      reg_response_val = get_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX, LSB);
      break;
    case MODBUS_REG_GAS_SUCTION_PX+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX, MSB);
      break;
    case MODBUS_REG_TC105:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC105, AS_INT);
      break;
    case MODBUS_REG_TC205:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC205, AS_INT);
      break;
    case MODBUS_REG_TC313:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC313, AS_INT);
      break;
    case MODBUS_REG_TC444:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC444, AS_INT);
      break;
    case MODBUS_REG_TC447:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC447, AS_INT);
      break;
    case MODBUS_REG_TC_RPSA1:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC_RPSA1, AS_INT);
      break;
    case MODBUS_REG_TC_RPSA2:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC_RPSA2, AS_INT);
      break;
    case MODBUS_REG_TC_SPARE:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_TC_SPARE, AS_INT);
      break;
    case MODBUS_REG_AO001:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_AO_1, AS_INT);
      break;
    case MODBUS_REG_AO002:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_AO_2, AS_INT);
      break;
    case MODBUS_REG_AO003:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_AO_3, AS_INT);
      break;
    case MODBUS_REG_AO004:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_AO_4, AS_INT);
      break;
    case MODBUS_REG_VFD_ON_OFF_SETTING:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_VFD_ON_OFF_SETTING, AS_INT);
      break;   
    case MODBUS_REG_VFD_SPEED_SETTING:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_VFD_SPEED_SETTING, AS_INT);
      break;         
    case MODBUS_REG_OP_STATE_ALL:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_OP_STATE_ALL, MSB);
      break;
    case MODBUS_REG_SLAVE_ID:
      reg_response_val = SLAVE_ID; 
      break; 
    case MODBUS_REG_PRODUCTION_EVO_DELTA_STEP:    // **** 32-bit float
      reg_response_val = get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP, LSB);
      break;
    case MODBUS_REG_PRODUCTION_EVO_DELTA_STEP+1:
      reg_response_val = get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP, MSB);
      break;
    case MODBUS_REG_PRODUCTION_EVO_DELTA_TIME:
      reg_response_val = (uint16_t)get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME, AS_INT);
      break;
    default:
      break;
  }
  return(reg_response_val);
}

int writeRegResponse(int register_n, int16_t value){
  #if defined(DEBUG_MODE)
    Serial.print(F("Write register: "));
    Serial.print(String(register_n));
    Serial.print(F(", value: "));
    Serial.println(String(value));
  #endif

  switch(register_n){
    case MODBUS_REG_BUTTONS_RGA:
      set_config_parameter(CONFIG_PARAM_OP_STATE_ALL, ((value >> OP_STATE_GREEN_BUTTON) & 0x1), OP_STATE_GREEN_BUTTON); 
      set_config_parameter(CONFIG_PARAM_OP_STATE_ALL, ((value >> OP_STATE_AMBER_BUTTON) & 0x1), OP_STATE_AMBER_BUTTON);   // according to Arduino?!??!
      set_config_parameter(CONFIG_PARAM_OP_STATE_ALL, ((value >> OP_STATE_RED_BUTTON) & 0x1), OP_STATE_RED_BUTTON);
      break;
    case MODBUS_REG_RELAYS:
      set_config_parameter(CONFIG_PARAM_RELAYS, value, MSB);
      break;
    case MODBUS_REG_RELAYS_2:
      set_config_parameter(CONFIG_PARAM_RELAYS, value, LSB);
      break;
    case MODBUS_REG_AO001:
      set_config_parameter(CONFIG_PARAM_AO_1, (float)value);
      break;
    case MODBUS_REG_AO002:
      set_config_parameter(CONFIG_PARAM_AO_2, (float)value);
      break;
    case MODBUS_REG_AO003:
      set_config_parameter(CONFIG_PARAM_AO_3, (float)value);
      break;
    case MODBUS_REG_AO004:
      set_config_parameter(CONFIG_PARAM_AO_4, (float)value);
      break;
    case MODBUS_REG_VFD_ON_OFF_SETTING:
      set_config_parameter(CONFIG_PARAM_VFD_ON_OFF_SETTING, (float)value);
      break;
    case MODBUS_REG_VFD_SPEED_SETTING:
      set_config_parameter(CONFIG_PARAM_VFD_SPEED_SETTING, (float)value);
      break;
    case MODBUS_REG_OP_STATE_ALL:
      // This prevents the user from changing non-writable operational state bits
      if (value & 0x1){
        set_config_bit(CONFIG_PARAM_OP_STATE_ALL, true, OP_STATE_MANUAL_CONTROL);
      }else{
        set_config_bit(CONFIG_PARAM_OP_STATE_ALL, false, OP_STATE_MANUAL_CONTROL);
      }
      break;
    case MODBUS_REG_PRODUCTION_EVO_DELTA_STEP:  // ****** 32-bit float
      set_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP, value, LSB);
      break;
    case MODBUS_REG_PRODUCTION_EVO_DELTA_STEP+1:
      set_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP, value, MSB);
      break;
    case MODBUS_REG_PRODUCTION_EVO_DELTA_TIME:
      set_config_parameter(MODBUS_REG_PRODUCTION_EVO_DELTA_TIME, (float)value);
      break;
    default:
      break;
  }
}

void modbuxRxData(uint8_t new_byte, int buffer)
{
  int * bufferPtr;

  if (buffer == TCP_BUFFER){
    TCPSerialBufferRx[TCPSerialBufferPt] = new_byte;
    bufferPtr = &TCPSerialBufferPt;
  }else if (buffer == RS_485_BUFFER){
    RSSerialBufferRx[RSSerialBufferPt] = new_byte;
    bufferPtr = &RSSerialBufferPt;
  }else if (buffer == BLE_BUFFER){
    BLESerialBufferRx[BLESerialBufferPt] = new_byte;
    bufferPtr = &BLESerialBufferPt;
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbuxRxData modbus_c200.cpp"));
    #endif
    return;
  }
  // Increment whichever pointer goes with this buffer
  if (*bufferPtr < (TCP_SERIAL_BUFFER_LIMIT - 1)){
    *bufferPtr = *bufferPtr + 1;    // The normal *buggerPtr++ will not work here. For some reason this needs to be explicit!
  }else{
    *bufferPtr = 0;
  }
  return;
}

int get_tx_bytes(int buffer)
{
  int temp_tx_bytes = 0;
  if (buffer == TCP_BUFFER){
    temp_tx_bytes = tcp_tx_bytes;
    tcp_tx_bytes = 0; 
  }else if (buffer == RS_485_BUFFER){
    temp_tx_bytes = rs_tx_bytes;
    rs_tx_bytes = 0; 
  }else if (buffer == BLE_BUFFER){
    temp_tx_bytes = ble_tx_bytes;
    ble_tx_bytes = 0; 
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_r050.cpp"));
    #endif
    return(0);
  }

  #if defined(DEBUG_MODE)
    Serial.print(F("Responding with n bytes: "));
    Serial.println(String(temp_tx_bytes));
  #endif
  return(temp_tx_bytes);
}

uint8_t * get_tx_buffer(int buffer)
{
  if (buffer == TCP_BUFFER){
    return(TCPSerialBufferTx);
  }else if (buffer == RS_485_BUFFER){
    return(RSSerialBufferTx);
  }else if (buffer == BLE_BUFFER){
    return(BLESerialBufferTx);
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_r050.cpp"));
    #endif
    return(0);
  }
}

// Investigate the buffer for valid MODBUS data and return a response
int modbus_loop(int buffer) {
  int dataLength = 8;   // All register reads are 8 bytes in length. ID, Fn, 2x REGISTER, 2x length req, 2x CRC = 8
  int * bufferPtr;
  uint8_t * processingBufferRx;
  uint8_t * processingBufferTx;
  int * tx_bytes;

  if (buffer == TCP_BUFFER){
    processingBufferRx = &TCPSerialBufferRx[0];
    processingBufferTx = &TCPSerialBufferTx[0];
    bufferPtr = &TCPSerialBufferPt;
    tx_bytes = &tcp_tx_bytes;
  }else if (buffer == RS_485_BUFFER){
    processingBufferRx = &RSSerialBufferRx[0];
    processingBufferTx = &RSSerialBufferTx[0];
    bufferPtr = &RSSerialBufferPt;
    tx_bytes = &rs_tx_bytes;
  }else if (buffer == BLE_BUFFER){
    processingBufferRx = &BLESerialBufferRx[0];
    processingBufferTx = &BLESerialBufferTx[0];
    bufferPtr = &BLESerialBufferPt;
    tx_bytes = &ble_tx_bytes;
  }else{
    #if defined(DEBUG_MODE)
      Serial.print(F("Unknown buffer in modbus_loop modbus_c200.cpp"));
    #endif
    return(0);
  }
  
  for(int i = 0; i < (TCP_SERIAL_BUFFER_LIMIT - dataLength); i++){  // Look through the entire buffer
    // For faster processing, don't bother doing anything unless the first byte under investigation is at least
    // the MOSBUS slave ID
    if (processingBufferRx[i] != SLAVE_ID){
      continue; 
    }

    int requestID = processingBufferRx[i]; // The ID in the packet from the master
    int functionCode = processingBufferRx[i+1]; // The function code. 
    int nRegisters = (processingBufferRx[i+4] << 8) | processingBufferRx[i+5];   // How many registers to read?
    int functionRegister = (processingBufferRx[i+2] << 8) | processingBufferRx[i+3];   // MB Register

    // The data length will depend on the function code we are operating with at this time
    if ((functionCode == 3) || (functionCode == 6)){
      dataLength = 6;   // All register reads are 8 bytes in length. ID, Fn, 2x REGISTER, 2x length req, 2x CRC = 8
    }else if (functionCode == 16){
      dataLength = 7 + (2*nRegisters);  // Total length 9 + 2*nR, however we do not include the CRC in the data length
    }
    
    if (i <= (TCP_SERIAL_BUFFER_LIMIT - dataLength)){         // If we have the entire packet
      if(CRC16(&processingBufferRx[i], dataLength, PACKET_CHECK_ONLY)){        // CRC Checking    
        // The tx buffer can only handle 255 bytes. So limit the number of registers we are allowed to read to a MAX 100
        if (nRegisters > 100){
            // Now that this packet has been processed - and is not valid for this device! We can destroy it so that it will not be processed again
            processingBufferRx[i] = 0;    // Wipe the RTU ID 
            processingBufferRx[i+1] = 0;  // Wipte the function code
            TCPSerialBufferPt = 0;   // We can return the pointer to the beginning of the buffer since a valid packet has been detected
            #if defined(DEBUG_MODE)
              Serial.println(F("Unable to return more than 100 registers at once"));
            #endif
        }else{ //  if (requestID == SLAVE_ID){ We already know this is true from above 
          if (functionCode == 3){
            int index = 0;    // Index the number of registers returned
            
            processingBufferTx[0] = 1;    // Slave ID
            processingBufferTx[1] = 3;    // Function code
            processingBufferTx[2] = 2 * nRegisters;    // Number of bytes being returned
            
            for (int j = nRegisters; j > 0; j--){
              uint16_t modbus_response_val = readRegResponse(functionRegister++);
              processingBufferTx[3+index] = (uint8_t)((modbus_response_val >> 8) & 0xFF);    // MSB data being returned
              processingBufferTx[4+index] = (uint8_t)(modbus_response_val & 0xFF);    // LSB data being returned, byte only 
              index = index + 2;  // Increment to the next point in the buffer where the register response will be stored for sending
            }
            // The data length is 3 (ID, FN, Length) + 2 bytes for every register!
            CRC16(processingBufferTx, (3 + (2*nRegisters)), PACKET_CRC_INSERT); // Insert the CRC
            
            // Finally, send the data
            //SerialBT.write(TCPSerialBufferTx, 7);  // Respond to the MODBUS request by sending data out of the BT Serial port
            *tx_bytes = 5 + (2 * nRegisters);
          }else if(functionCode == 16){
            int16_t value_n = (processingBufferRx[i+7] << 8) | processingBufferRx[i+8];   // MB Register
            Serial.print(F("Function code 16 CRC Pass for nRegisters = "));
            Serial.println(nRegisters);            
            
            processingBufferTx[0] = 1;    // Slave ID
            processingBufferTx[1] = 16;    // Function code
            processingBufferTx[2] = processingBufferRx[i+2];    // MB register MSB
            processingBufferTx[3] = processingBufferRx[i+3];   // MB Register LSB
            processingBufferTx[4] = processingBufferRx[i+4];    // n registers MSB
            processingBufferTx[5] = processingBufferRx[i+5];    // n registers LSB
            CRC16(processingBufferTx, 6, PACKET_CRC_INSERT); // Insert the CRC

            while(nRegisters > 0){
              writeRegResponse(functionRegister, value_n);    // Write the value
              nRegisters--;   // Remove one from the number of registers we need to process
              functionRegister++;   // Move to the next function register to be written
              i = i + 2;    // Move our buffer indexing to the next word (below) that will be written
              value_n = (processingBufferRx[i+7] << 8) | processingBufferRx[i+8];   // Next value to be written
            }

            // Finally, send the data
            //SerialBT.write(TCPSerialBufferTx, 7);  // Respond to the MODBUS request by sending data out of the BT Serial port
            *tx_bytes = 8;
          }else{   // End if read holding registers
            // Unsupported function code
            Serial.print(F("Unsupported function code: "));
            Serial.println(String(functionCode));
          }// End if test function code
          // Wipe the data bytes just processed in the buffer to avoid double processing
          memset(processingBufferRx, '\0', TCP_SERIAL_BUFFER_LIMIT);    // We know that dataLength is safe because it was tested above
          *bufferPtr = 0; // Reset the buffer data pointer to the beginning of the buffer
        } // End if slave ID matches our own
      } // end CRC checking
    }
  } // End buffer itteration
  return(*tx_bytes);
}
