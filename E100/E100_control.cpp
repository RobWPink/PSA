#include <Arduino.h>
#include "config_e100.h"
#include "io_e100.h"
//red button needs to be set for bluetooth reset

#define SPTR_SIZE   20

enum stt{
  IDLE,
  R050_OK,
  CHECK_EVO,
  INIT_EVO,
  RAMP_DOWN,
  RAMP_UP,
  BALANCE_EVO,
  SHUTDOWN,
  ESTOP
}STATE,PREV;


unsigned long timer = 0;
unsigned long flashTimer[3] = {0};
unsigned long debouncer[6] = {0};
bool flashTog[3] = {false};
bool heater = false;
bool overLvl = false;
double mxy = 0;
bool greenButton, redButton = false;
String errString = "";
int flashGreen, flashAmber, flashRed = 0;

void flashDriver(){
  if (!flashTimer[0] && flashGreen) { flashTimer[0] = millis(); }
  if (millis() - flashTimer[0] > flashGreen && flashGreen) {
    flashTog[0] = !flashTog[0];
    set_config_bit(CONFIG_PARAM_RELAYS,flashTog[0],RELAY_BIT_GREEN_PILOT);
    flashTimer[0] = millis();
  }
  else if (!flashGreen && flashTimer[0]) {
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_GREEN_PILOT);
    flashTimer[0] = 0;
  }

  if (!flashTimer[1] && flashAmber) { flashTimer[1] = millis(); }
  if (millis() - flashTimer[1] > flashAmber && flashAmber) {
    flashTog[1] = !flashTog[1];
    set_config_bit(CONFIG_PARAM_RELAYS,flashTog[1],RELAY_BIT_AMBER_PILOT);
    flashTimer[1] = millis();
  }
  else if (!flashAmber && flashTimer[1]) {
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_AMBER_PILOT);
    flashTimer[1] = 0;
  }

  if (!flashTimer[2] && flashRed) { flashTimer[2] = millis(); }
  if (millis() - flashTimer[2] > flashRed && flashRed) {
    flashTog[2] = !flashTog[2];
    set_config_bit(CONFIG_PARAM_RELAYS,flashTog[2],RELAY_BIT_RED_PILOT);
    flashTimer[2] = millis();
  }
  else if (!flashRed && flashTimer[2]) {
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_RED_PILOT);
    flashTimer[2] = 0;
  }
}

void waterDropoutControl(){

  if(!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_HIGH_LEVEL_DO_1)){if(!debouncer[0]){debouncer[0] = millis();}} //LSH's are inverted, LOW = water, HIGH = no water
  else{debouncer[0] = 0;}
  if(millis() - debouncer[0] > 300 && debouncer[0]){
    set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_VE311);
    debouncer[0] = 0;
  }
  if(!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LOW_LEVEL_DO_1)){if(!debouncer[1]){debouncer[1] = millis();}}
  else{debouncer[1] = 0;}
  if(millis() - debouncer[1] > 300 && debouncer[1]){
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_VE311);
    debouncer[1] = 0;
  }
  if(!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_HIGH_LEVEL_DO_2)){if(!debouncer[2]){debouncer[2] = millis();}}
  else{debouncer[2] = 0;}
  if(millis() - debouncer[2] > 300 && debouncer[2]){
    set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_VE511);
    debouncer[2] = 0;
  }
  if(!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LOW_LEVEL_DO_2)){if(!debouncer[3]){debouncer[3] = millis();}}
  else{debouncer[3] = 0;}
  if(millis() - debouncer[3] > 300 && debouncer[3]){
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_VE511);
    debouncer[3] = 0;
  }
}

bool oilTempControl(){
  if(get_config_parameter(CONFIG_PARAM_TC447) < 70){
    heater = true;
    set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_DIVERTER_XV);
  }
  else if(get_config_parameter(CONFIG_PARAM_TC447) > 85){
    heater = false;
    set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_DIVERTER_XV);
  }
  if(heater){
    if(get_config_parameter(CONFIG_PARAM_TC444) > 120){set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_OIL_HEATER);}
    else if(get_config_parameter(CONFIG_PARAM_TC444) < 70){set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_OIL_HEATER);}
  }
}
bool operationCheck(){
  return true;
}


void E100control(){
  bool oilOK = oilTempControl();

  flashDriver();

  waterDropoutControl();

  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_GREEN_BUTTON)){if(!debouncer[4]){debouncer[4] = millis();}}
  else{debouncer[4] = 0;greenButton = false;}
  if(millis() - debouncer[4] > 300 && debouncer[4]){
    greenButton = true;
    debouncer[4] = 0;
  }

  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_RED_BUTTON)){if(!debouncer[5]){debouncer[5] = millis();}}
  else{debouncer[5] = 0;redButton = false;}
  if(millis() - debouncer[5] > 300 && debouncer[5]){
    redButton = true;
    debouncer[5] = 0;
  }

  if(redButton){
    errString = "Manual Shutdown Detected.";
    STATE = SHUTDOWN;
  }

  if(STATE > R050_OK && !test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READY_IP_R050_1)){
    errString = "Lost OK signal from Reformer 1.";
    STATE = SHUTDOWN;
  }

  set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_LOCAL_INTERLOCK);

  set_config_bit(CONFIG_PARAM_RELAYS,!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSR_IP),RELAY_BIT_AMBER_PILOT);

  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LOCAL_ESTOP)){
    errString = "ESTOP Detected!";
    STATE = ESTOP;
  }

  if(STATE != PREV){
    timer = 0;
    PREV = STATE;
  }
  switch(STATE){
    case IDLE:
      set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_GREEN_PILOT);
      if(greenButton && !test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSR_IP) && oilOK){ STATE = R050_OK; }
    break;
		
    case R050_OK:
      flashGreen = 250;
      if(get_config_parameter(CONFIG_PARAM_TC447) < 25){
        set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_READY_R050_1);
        set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_PSA1_VFD_RUN);
        set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_XV_RPSA1_GAS_FEED);
      }
      else if(get_config_parameter(CONFIG_PARAM_TC447) > 25){
        if(operationCheck()){
          set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_READY_R050_1);
          set_config_bit(CONFIG_PARAM_RELAYS,test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READY_IP_R050_1),RELAY_BIT_PSA1_VFD_RUN);
          set_config_parameter(CONFIG_PARAM_AO_3,83); // 50/60Hz
          set_config_bit(CONFIG_PARAM_RELAYS,test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READY_IP_R050_1),RELAY_BIT_XV_RPSA1_GAS_FEED);
        }
        else{
          set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_READY_R050_1);
          set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_PSA1_VFD_RUN);
          set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_XV_RPSA1_GAS_FEED);
        }
      }
      if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_READY_IP_R050_1) &&
        test_config_parameter(CONFIG_PARAM_RELAYS,RELAY_BIT_READY_R050_1)){ STATE = CHECK_EVO; }
    break;

    case CHECK_EVO:
      flashGreen = 500;
      if(get_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX) > 15){ STATE = INIT_EVO; }
    break;

    case INIT_EVO:
      if(!timer){
        timer = millis();
        set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_EVO_VFD_RUN);
        set_config_parameter(CONFIG_PARAM_AO_4,20);
        set_config_parameter(CONFIG_PARAM_AO_1,75);
      }
      if(millis() - timer > 3000 && timer){ STATE = RAMP_DOWN; }
    break;
    
    case RAMP_DOWN:
      flashGreen = 750;
      if(!timer){timer = millis();}
      mxy = ((double)-50/120000)*(millis() - timer)+75.0;
      if(mxy >= 25.0){set_config_parameter(CONFIG_PARAM_AO_1,mxy);}
      if(millis() - timer > (double)60000*3 && timer){ STATE = RAMP_UP; }
    break;

    case RAMP_UP:
      flashGreen = 1000;
      if(!timer){timer = millis();}
        mxy = ((double)50/300000)*(millis() - timer)+25.0;
        if(mxy <= 75.0){set_config_parameter(CONFIG_PARAM_AO_1,mxy);}
        if(millis() - timer > (double)60000*5 && timer){ STATE = BALANCE_EVO; }
    break;

    case BALANCE_EVO:
      flashGreen = 0;
      set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_GREEN_PILOT);
      if(!timer){timer = millis();}
      if(millis() - timer > (unsigned long)get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME) && timer){
        if(get_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX) > 9){
          set_config_parameter(CONFIG_PARAM_AO_4,get_config_parameter(CONFIG_PARAM_AO_4)+(double)get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP));
        }
        else if(get_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX) < 8){
          set_config_parameter(CONFIG_PARAM_AO_4,get_config_parameter(CONFIG_PARAM_AO_4)-(double)get_config_parameter(CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP));
        }
        timer = millis();
      }
      
    break;

    case SHUTDOWN:
      if(!timer){
        timer = millis();
        flashGreen = 0;
        set_config_parameter(CONFIG_PARAM_RELAYS,0);
        set_config_parameter(CONFIG_PARAM_AO_1,0);
        set_config_parameter(CONFIG_PARAM_AO_2,0);
        set_config_parameter(CONFIG_PARAM_AO_3,0);
        set_config_parameter(CONFIG_PARAM_AO_4,0);
        set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_LOCAL_INTERLOCK);
        // Serial.println(errString); // Debug use
      }
      set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_RED_PILOT);
      if(millis() - timer > 3000 && timer){ STATE = IDLE; set_config_bit(CONFIG_PARAM_RELAYS,false,RELAY_BIT_RED_PILOT);}
    break;

    case ESTOP:
      if(!timer){
        timer = millis();
        flashGreen = 0;
        set_config_parameter(CONFIG_PARAM_RELAYS,0);
        set_config_parameter(CONFIG_PARAM_AO_1,0);
        set_config_parameter(CONFIG_PARAM_AO_2,0);
        set_config_parameter(CONFIG_PARAM_AO_3,0);
        set_config_parameter(CONFIG_PARAM_AO_4,0);
        set_config_bit(CONFIG_PARAM_RELAYS,true,RELAY_BIT_LOCAL_INTERLOCK);
        // Serial.println(errString); // Debug use
      }
      flashRed = 250;
      if(!test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LOCAL_ESTOP)){
        flashRed = 0;
        STATE = IDLE;
      }
    break;

    default:
    break;
  }
}
