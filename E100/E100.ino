#include "E100.h"

double TT447, TT444, PT454;
double HTR443_time,HEATER_time,EVO_time;
double TT444_set = 100;
double TT447_set = 83;
double PT454_set = 8.5;

PID heatingElementPID(&TT447, &HTR443_time, &TT444_set, 2, 0, 0, DIRECT);
PID oilTempPID(&TT444, &HEATER_time, &TT447_set, 2, 0, 0, DIRECT);
PID evoSpeedPID(&PT454, &EVO_time, &PT454_set, 1, 0, 0, DIRECT);

void setup() {
 Serial.begin(9600);

}

void loop() {
  oilTempControl();
  waterDropoutControl();


  switch(STATE){
    case R050_OK:
      if(get_config_parameter(PARAM_TT447) < 75){
        set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_READYOP1);
        set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_READYOP2);
        set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_RPSA1_VFD_RUN);
        set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RPSA1_GAS_XV);
      }
      else if(get_config_parameter(PARAM_TT447) > 80){
          if(operationCheck()){
            set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_READYOP1);
            set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_READYOP2);
            set_config_bit(CONFIG_PARAM_RELAYS,test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READYIP1),RELAY_BIT_RPSA1_VFD_RUN);
            set_config_bit(CONFIG_PARAM_RELAYS,test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READYIP1),RPSA1_GAS_XV);
          }
          else{
            set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_READYOP1);
            set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_READYOP2);
            set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_RPSA1_VFD_RUN);
            set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RPSA1_GAS_XV);
          }
        }
      }
      if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READYIP1) && 
         test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READYIP2) &&
         test_config_parameter(CONFIG_PARAM_RELAYS,RELAY_BIT_READYOP1) &&
         test_config_parameter(CONFIG_PARAM_RELAYS,RELAY_BIT_READYOP2)){
        STATE = INIT_EVO;
      }
    case INIT_EVO:
      double mxy = 0;
      if(test_config_parameter(CONFIG_PARAM_RELAYS,RELAY_BIT_READYOP1)){
        if(get_config_parameter(PARAM_SUCTIONPSI) > 15){
          set_config_parameter(EVOHz,20);
        }
        if(get_config_parameter(PARAM_EVOHz) == 20){
          if(!timer[0]){timer[0] = millis();}
          else if(millis() - timer[0] < 60000*2 && timer[0]){ //decrease ACT1 pos from 75% to 25% over 2min
            mxy = (double)((-1*timer[0]/2400)+75)/100;
            if(mxy >= 0.25){set_config_parameter(ACT1_POS,ACT1_MAX*mxy)};
          } //hold ACT1 pos for 1min
          else if(60000*3 < millis() - timer[0] < 60000*8){ //increase ACT1 pos from 25% to 75% over 5min
            mxy = (double)((-1*timer[0]/6000)+25)/100;
            if(mxy <= 0.75){set_config_parameter(ACT1_POS,ACT1_MAX*mxy)};
          }
          else if(millis() - timer[0] > 60000*8){
            STATE = BALANCE_EVO;
          }
        }
      }
    break;

    case BALANCE_EVO:
      if(get_config_parameter(PARAM_SUCTIONPSI) > 9){set_config_parameter(EVOHz,get_config_parameter(PARAM_EVOHz)++);}
      else if(get_config_parameter(PARAM_SUCTIONPSI) < 8){set_config_parameter(EVOHz,get_config_parameter(PARAM_EVOHz)++);}//maybe use a PID controller here
  }

}

