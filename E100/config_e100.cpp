#include <stdint.h>
#include <Wire.h>
#include "config_e100.h"
#include "eeprom.h"

static FLOATUNION_t relays, inputs, operational_states; 

// Instances of the union to store the min and max values for various modes (stop, idle, run)
// This defines the operational limits for each mode
static FLOATUNION_t EVO_GAS_PX, EVO_GAS_STOP_MIN, EVO_GAS_IDLE_MIN, EVO_GAS_RUN_MIN, EVO_GAS_RUN_MAX, EVO_GAS_IDLE_MAX, EVO_GAS_STOP_MAX;                               //AI.1, TB44
static FLOATUNION_t PSA_PRESS_1_PX, PSA_PRESS_1_STOP_MIN, PSA_PRESS_1_IDLE_MIN, PSA_PRESS_1_RUN_MIN, PSA_PRESS_1_RUN_MAX, PSA_PRESS_1_IDLE_MAX, PSA_PRESS_1_STOP_MAX;   //AI.2, TB45
static FLOATUNION_t PSA_PRESS_2_PX, PSA_PRESS_2_STOP_MIN, PSA_PRESS_2_IDLE_MIN, PSA_PRESS_2_RUN_MIN, PSA_PRESS_2_RUN_MAX, PSA_PRESS_2_IDLE_MAX, PSA_PRESS_2_STOP_MAX;   //AI.3, TB46
static FLOATUNION_t FINAL_GAS_DISCHARGE_PX, FINAL_GAS_DISCHARGE_STOP_MIN, FINAL_GAS_DISCHARGE_IDLE_MIN, FINAL_GAS_DISCHARGE_RUN_MIN, 
                    FINAL_GAS_DISCHARGE_RUN_MAX, FINAL_GAS_DISCHARGE_IDLE_MAX, FINAL_GAS_DISCHARGE_STOP_MAX;                                                            //AI.4, TB47
static FLOATUNION_t ACT_1_PX, ACT_1_STOP_MIN, ACT_1_IDLE_MIN, ACT_1_RUN_MIN, ACT_1_RUN_MAX, ACT_1_IDLE_MAX, ACT_1_STOP_MAX;                                             // AI.5 TB48
static FLOATUNION_t ACT_2_PX, ACT_2_STOP_MIN, ACT_2_IDLE_MIN, ACT_2_RUN_MIN, ACT_2_RUN_MAX, ACT_2_IDLE_MAX, ACT_2_STOP_MAX;                                             //AI.6 TB49
static FLOATUNION_t PSA_1_PX, PSA_1_STOP_MIN, PSA_1_IDLE_MIN, PSA_1_RUN_MIN, PSA_1_RUN_MAX, PSA_1_IDLE_MAX, PSA_1_STOP_MAX;                                             // AI.7 TB50
static FLOATUNION_t GAS_SUCTION_PX, GAS_SUCTION_STOP_MIN, GAS_SUCTION_IDLE_MIN, GAS_SUCTION_RUN_MIN, GAS_SUCTION_RUN_MAX, GAS_SUCTION_IDLE_MAX, GAS_SUCTION_STOP_MAX;   //AI.8 TB51

static FLOATUNION_t tc105, tc205, tc313, tc444, tc447, tc_rpsa1, tc_rpsa2, tc_spare;
static FLOATUNION_t I2C_AO_1, I2C_AO_2, I2C_AO_3, I2C_AO_4;

static FLOATUNION_t EVI_1_Delta_Step, EVO1_Delta_Time;

static FLOATUNION_t vfd_485_on_off, vfd_485_speed;


void init_config()
{
  relays.number = 0;      // Set parameter initial values. 
  inputs.number = 0;      // Digital inputs
  operational_states.number = 0; 
  tc105.number = -1; 
  tc205.number = -1; 
  tc313.number = -1; 
  tc444.number = -1; 
  tc447.number = -1; 
  tc_rpsa1.number = -1; 
  tc_rpsa2.number = -1; 
  tc_spare.number = -1;
  vfd_485_on_off.number = 0; 
  vfd_485_speed.number = 0; 

  // Initialize these values, however they are read from sensor

  // EVO GAS PRESSURE PT SETPOINTS
  EVO_GAS_PX.number = -1; 
  EVO_GAS_STOP_MIN.number= SETPOINT_EVO_GAS_ENTER_STOP_MIN;           //Initial set point values need to be changed in config.h once confirmed 
  EVO_GAS_IDLE_MIN.number= SETPOINT_EVO_GAS_ENTER_IDLE_MIN;           //only for future use
  EVO_GAS_RUN_MIN.number=  SETPOINT_EVO_GAS_ENTER_RUN_MIN;
  EVO_GAS_RUN_MAX.number=  SETPOINT_EVO_GAS_ENTER_RUN_MAX;
  EVO_GAS_IDLE_MAX.number= SETPOINT_EVO_GAS_ENTER_IDLE_MAX;
  EVO_GAS_STOP_MAX.number= SETPOINT_EVO_GAS_ENTER_STOP_MAX;

  //PSA PRESS 1 PRESSURE PT SETPOINTS
  PSA_PRESS_1_PX.number= -1;
  PSA_PRESS_1_STOP_MIN.number=  SETPOINT_PSA_PRESS_1_ENTER_STOP_MIN;   //Initial set point values need to be changed in config.h once confirmed 
  PSA_PRESS_1_IDLE_MIN.number=  SETPOINT_PSA_PRESS_1_ENTER_IDLE_MIN;   //only for future use
  PSA_PRESS_1_RUN_MIN.number=   SETPOINT_PSA_PRESS_1_ENTER_RUN_MIN;
  PSA_PRESS_1_RUN_MAX.number=   SETPOINT_PSA_PRESS_1_ENTER_RUN_MAX;
  PSA_PRESS_1_IDLE_MAX.number=  SETPOINT_PSA_PRESS_1_ENTER_IDLE_MAX;
  PSA_PRESS_1_STOP_MAX.number=  SETPOINT_PSA_PRESS_1_ENTER_STOP_MAX;

  //PSA PRESS 2 PRESSIRE PT SETPOINTS
  PSA_PRESS_2_PX.number= -1;
  PSA_PRESS_2_STOP_MIN.number= SETPOINT_PSA_PRESS_2_ENTER_STOP_MIN;     //Initial set point values need to be changed in config.h once confirmed 
  PSA_PRESS_2_IDLE_MIN.number= SETPOINT_PSA_PRESS_2_ENTER_IDLE_MIN;     //only for future use
  PSA_PRESS_2_RUN_MIN.number=  SETPOINT_PSA_PRESS_2_ENTER_RUN_MIN;
  PSA_PRESS_2_RUN_MAX.number=  SETPOINT_PSA_PRESS_2_ENTER_RUN_MAX;
  PSA_PRESS_2_IDLE_MAX.number= SETPOINT_PSA_PRESS_2_ENTER_IDLE_MAX;
  PSA_PRESS_2_STOP_MAX.number= SETPOINT_PSA_PRESS_2_ENTER_STOP_MAX;

  FINAL_GAS_DISCHARGE_PX.number= -1;
  ACT_1_PX.number= -1;
  ACT_2_PX.number= -1;
  PSA_1_PX.number= -1;
  GAS_SUCTION_PX.number= -1;

  EVI_1_Delta_Step.number = 1;     // Default values
  EVO1_Delta_Time.number = 250;
}

float get_config_parameter(int param){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      return(relays.number);
    case CONFIG_PARAM_INPUTS:
      Serial.print("Inputs CF: ");
      Serial.println(inputs.number);
      return(inputs.number);
    //EVO GAS PRESSURE GET CONFIG VALUES 
    case CONFIG_PARAM_EVO_GAS_PX:
      return (EVO_GAS_PX.number);
    case CONFIG_PARAM_EVO_GAS_STOP_MIN:
      return(EVO_GAS_STOP_MIN.number);
    case CONFIG_PARAM_EVO_GAS_IDLE_MIN:
      return(EVO_GAS_IDLE_MIN.number);
    case CONFIG_PARAM_EVO_GAS_RUN_MIN:
      return(EVO_GAS_RUN_MIN.number);
    case CONFIG_PARAM_EVO_GAS_RUN_MAX:
      return(EVO_GAS_RUN_MAX.number);
    case CONFIG_PARAM_EVO_GAS_IDLE_MAX:
      return(EVO_GAS_IDLE_MAX.number);
    case CONFIG_PARAM_EVO_GAS_STOP_MAX:
      return(EVO_GAS_STOP_MAX.number); 

    // PSA PRESS 1 PRESSURE GET VALUES
    case CONFIG_PARAM_PSA_PRESS_1_PX:
      return (PSA_PRESS_1_PX.number);
    case CONFIG_PARAM_PSA_PRESS_1_STOP_MIN:
      return(PSA_PRESS_1_STOP_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_1_IDLE_MIN:
      return(PSA_PRESS_1_IDLE_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_1_RUN_MIN:
      return(PSA_PRESS_1_RUN_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_1_RUN_MAX:
      return(PSA_PRESS_1_RUN_MAX.number);
    case CONFIG_PARAM_PSA_PRESS_1_IDLE_MAX:
      return(PSA_PRESS_1_IDLE_MAX.number);
    case CONFIG_PARAM_PSA_PRESS_1_STOP_MAX:
      return(PSA_PRESS_1_STOP_MAX.number); 

    //PSA PRESS 2 RETURN CONFIG VALUES
    case CONFIG_PARAM_PSA_PRESS_2_PX:
      return (PSA_PRESS_2_PX.number);
    case CONFIG_PARAM_PSA_PRESS_2_STOP_MIN:
      return(PSA_PRESS_2_STOP_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_2_IDLE_MIN:
      return(PSA_PRESS_2_IDLE_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_2_RUN_MIN:
      return(PSA_PRESS_2_RUN_MIN.number);
    case CONFIG_PARAM_PSA_PRESS_2_RUN_MAX:
      return(PSA_PRESS_2_RUN_MAX.number);
    case CONFIG_PARAM_PSA_PRESS_2_IDLE_MAX:
      return(PSA_PRESS_2_IDLE_MAX.number);
    case CONFIG_PARAM_PSA_PRESS_2_STOP_MAX:
      return(PSA_PRESS_2_STOP_MAX.number); 

    //yet to be completed
    case CONFIG_PARAM_FINAL_GAS_DISCHARGE_PX:
      return (FINAL_GAS_DISCHARGE_PX.number);
    case CONFIG_PARAM_ACT_1_PX:
      return (ACT_1_PX.number);
    case CONFIG_PARAM_ACT_2_PX:
      return(ACT_2_PX.number);
    case CONFIG_PARAM_PSA_1_PX:
      return(PSA_1_PX.number);
    case CONFIG_PARAM_GAS_SUCTION_PX:
      return(GAS_SUCTION_PX.number);
    case CONFIG_PARAM_TC105:
      return(tc105.number);
    case CONFIG_PARAM_TC205:
      return(tc205.number);
    case CONFIG_PARAM_TC313:
      return(tc313.number);
    case CONFIG_PARAM_TC444:
      return(tc444.number);
    case CONFIG_PARAM_TC447:
      return(tc447.number);
    case CONFIG_PARAM_TC_RPSA1:
      return(tc_rpsa1.number);
    case CONFIG_PARAM_TC_RPSA2:
      return(tc_rpsa2.number);
    case CONFIG_PARAM_TC_SPARE:
      return(tc_spare.number);

    //analog outputs
    case CONFIG_PARAM_AO_1:
      return(I2C_AO_1.number);
    case CONFIG_PARAM_AO_2:
      return(I2C_AO_2.number);
    case CONFIG_PARAM_AO_3:
      return(I2C_AO_3.number);
    case CONFIG_PARAM_AO_4:
      return(I2C_AO_4.number);
    case CONFIG_PARAM_VFD_ON_OFF_SETTING:
      return(vfd_485_on_off.number);
    case CONFIG_PARAM_VFD_SPEED_SETTING:
      return(vfd_485_speed.number);
    // *****   Configs *******
    case CONFIG_PARAM_OP_STATE_ALL:
      return(operational_states.number);
    case CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP:
      return(EVI_1_Delta_Step.number);
    case CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME:
      return(EVO1_Delta_Time.number);
    default:
      return(-1);
      break;
  }
}

bool test_config_parameter(int param, int mask){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      return(relays.uint_value & (1 << mask));
    case CONFIG_PARAM_INPUTS:
      return(inputs.uint_value & (1 << mask));
    case CONFIG_PARAM_OP_STATE_ALL:
      return(operational_states.uint_value & (1 << mask));
    default:
      return(0);
  }  
}

uint16_t get_config_parameter(int param, int byte_n){
  uint16_t return_param = 0;
  FLOATUNION_t return_float;
  return_float.number = get_config_parameter(param);

  if (byte_n == MSB){
    return_param = return_float.words[0];
  }else if (byte_n == LSB){
    return_param = return_float.words[1];
  }else if (byte_n == AS_INT){    // As int
    return_param =  int(return_float.number);
  }
  return(return_param);
}

void set_config_parameter(int param, float param_value){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      relays.number = param_value;
      break;
    case CONFIG_PARAM_INPUTS:
      inputs.number = param_value;
      break;
    //EVO GAS DISCHARGE PRESSURE SET CONFIG  
    case CONFIG_PARAM_EVO_GAS_PX:
      EVO_GAS_PX.number= param_value;
      break;
    case CONFIG_PARAM_EVO_GAS_STOP_MIN:
      EVO_GAS_STOP_MIN.number = param_value;          // EVO GAS PRESSURE trigger point for STOP mode (low/minimum)
      break;
    case CONFIG_PARAM_EVO_GAS_IDLE_MIN:
      EVO_GAS_IDLE_MIN.number = param_value;          // trigger point for IDLE mode (low/minimum)
      break;
    case CONFIG_PARAM_EVO_GAS_RUN_MIN: 
      EVO_GAS_RUN_MIN.number = param_value;          // trigger point for return to RUN mode (low/minimum)
      break;
    case CONFIG_PARAM_EVO_GAS_RUN_MAX:
      EVO_GAS_RUN_MAX.number = param_value;          // trigger point for return to RUN mode (high/maximum)
      break;
    case CONFIG_PARAM_EVO_GAS_IDLE_MAX:
      EVO_GAS_IDLE_MAX.number = param_value;          // trigger point for return to IDLE mode (high/maximum)
      break;
    case CONFIG_PARAM_EVO_GAS_STOP_MAX:
      EVO_GAS_STOP_MAX.number = param_value;          // trigger point for return to STOP mode (high/maximum)
      break;

    // PSA PRESS 1 PRESSURE SET CONFIG
    case CONFIG_PARAM_PSA_PRESS_1_PX:
      PSA_PRESS_1_PX.number= param_value;
      break;
    case CONFIG_PARAM_PSA_PRESS_1_STOP_MIN:
      PSA_PRESS_1_STOP_MIN.number = param_value;          // EVO GAS PRESSURE trigger point for STOP mode (low/minimum)
      break;
    case CONFIG_PARAM_PSA_PRESS_1_IDLE_MIN:
      PSA_PRESS_1_IDLE_MIN.number = param_value;          // trigger point for IDLE mode (low/minimum)
      break;
    case CONFIG_PARAM_PSA_PRESS_1_RUN_MIN: 
      PSA_PRESS_1_RUN_MIN.number = param_value;          // trigger point for return to RUN mode (low/minimum)
      break;
    case CONFIG_PARAM_PSA_PRESS_1_RUN_MAX:
      PSA_PRESS_1_RUN_MAX.number = param_value;          // trigger point for return to RUN mode (high/maximum)
      break;
    case CONFIG_PARAM_PSA_PRESS_1_IDLE_MAX:
      PSA_PRESS_1_IDLE_MAX.number = param_value;          // trigger point for return to IDLE mode (high/maximum)
      break;
    case CONFIG_PARAM_PSA_PRESS_1_STOP_MAX:
      PSA_PRESS_1_STOP_MAX.number = param_value;          // trigger point for return to STOP mode (high/maximum)
      break;

    case CONFIG_PARAM_PSA_PRESS_2_PX:
      PSA_PRESS_2_PX.number= param_value;
      break;
    case CONFIG_PARAM_FINAL_GAS_DISCHARGE_PX:
      FINAL_GAS_DISCHARGE_PX.number= param_value;
      break;
    case CONFIG_PARAM_ACT_1_PX:
      ACT_1_PX.number= param_value;
      break;
    case CONFIG_PARAM_ACT_2_PX:
      ACT_2_PX.number= param_value;
      break;
    case CONFIG_PARAM_PSA_1_PX:
      PSA_1_PX.number= param_value;
      break;
    case CONFIG_PARAM_GAS_SUCTION_PX:
      GAS_SUCTION_PX.number= param_value;
      break;
    case CONFIG_PARAM_TC105:
      tc105.number = param_value;
      break;
    case CONFIG_PARAM_TC205:
      tc205.number = param_value;
      break;
    case CONFIG_PARAM_TC313:
      tc313.number = param_value;
      break;
    case CONFIG_PARAM_TC444:
      tc444.number = param_value;
      break;
    case CONFIG_PARAM_TC447:
      tc447.number = param_value;
      break;
    case CONFIG_PARAM_TC_RPSA1:
      tc_rpsa1.number = param_value;
      break;
    case CONFIG_PARAM_TC_RPSA2:
      tc_rpsa2.number = param_value;
      break;
    case CONFIG_PARAM_TC_SPARE:
      tc_spare.number = param_value;
      break;
    //analog output set 
    case CONFIG_PARAM_AO_1:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      I2C_AO_1.number = param_value;
      break; 
    case CONFIG_PARAM_AO_2:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      I2C_AO_2.number = param_value;
      break;
    case CONFIG_PARAM_AO_3:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      I2C_AO_3.number = param_value;
      break;
    case CONFIG_PARAM_AO_4:
      if (param_value > 100){
        param_value = 100; 
      }
      if (param_value < 0){
        param_value = 0;
      }
      I2C_AO_4.number = param_value;
      break;
    case CONFIG_PARAM_VFD_ON_OFF_SETTING:
      vfd_485_on_off.number = param_value;
      break;
    case CONFIG_PARAM_VFD_SPEED_SETTING:
      vfd_485_speed.number = param_value;
      break;
    // *****   Configs *******
    case CONFIG_PARAM_OP_STATE_ALL:
      Serial.print("Write value: ");
      Serial.println(param_value);
      operational_states.number = param_value;
      break;
    case CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP:
      eeprom_save();  // Save changes to this variable in persistent memory
      EVI_1_Delta_Step.number = param_value;
      break;
    case CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME:
      eeprom_save();  // Save changes to this variable in persistent memory
      EVO1_Delta_Time.number = param_value; 
      break;
    default:
      break;
  }
}

void set_config_bit(int param, uint16_t param_value, int bit_n){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      if (param_value){
        relays.uint_value |= 1 << bit_n;
      }else{
        relays.uint_value &= ~(1 << bit_n);
      }
      break;
    case CONFIG_PARAM_INPUTS:
      if (param_value){
        inputs.uint_value |= 1 << bit_n;
      }else{
        inputs.uint_value &= ~(1 << bit_n);
      }
      break;
    case CONFIG_PARAM_OP_STATE_ALL:
      if (param_value){
        operational_states.uint_value |= 1 << bit_n;
      }else{
        operational_states.uint_value &= ~(1 << bit_n);
      }
      break;
  }
}

void set_config_parameter(int param, uint16_t param_value, int byte_n){
  switch(param){
    case CONFIG_PARAM_RELAYS:
      if (byte_n == MSB){
        relays.words[0] = param_value;
      }else{
        relays.words[1] = param_value;
      }
      break;
    case CONFIG_PARAM_INPUTS:
      if (param_value){
        inputs.uint_value |= 1 << byte_n;
      }else{
        inputs.uint_value &= ~(1 << byte_n);
      }
      break;
    case CONFIG_PARAM_OP_STATE_ALL:
      if (param_value){
        operational_states.uint_value |= 1 << byte_n;
      }else{
        operational_states.uint_value &= ~(1 << byte_n);
      }
      break;
    // Setting of a 32-bit float using 2x 16-bit Modbus registers
    case CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP:
      if (byte_n == MSB){
        EVI_1_Delta_Step.words[0] = param_value;
      }else{
        EVI_1_Delta_Step.words[1] = param_value;
      }
      eeprom_save();  // Save changes to this variable in persistent memory
      break;
    default:
      break;
  }
}
