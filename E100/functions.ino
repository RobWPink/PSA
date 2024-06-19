void waterDropoutControl(){
  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSH321)){if(!debouncer[0]){debouncer[0] = millis();}
  else{debouncer[0] = 0;}
  if(millis() - debouncer[0] > 300 && debouncer[0]){
    set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_VE311);
    debouncer[0] = 0;
  }
  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSL322)){if(!debouncer[1]){debouncer[1] = millis();}
  else{debouncer[1] = 0;}
  if(millis() - debouncer[1] > 300 && debouncer[1]){
    set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_VE311);
    debouncer[1] = 0;
  }
  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSH521)){if(!debouncer[2]){debouncer[2] = millis();}
  else{debouncer[2] = 0;}
  if(millis() - debouncer[2] > 300 && debouncer[2]){
    set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_VE511);
    debouncer[2] = 0;
  }
  if(test_config_parameter(CONFIG_PARAM_INPUTS,INPUT_BIT_LSL522)){if(!debouncer[3]){debouncer[3] = millis();}
  else{debouncer[3] = 0;}
  if(millis() - debouncer[3] > 300 && debouncer[3]){
    set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_VE511);
    debouncer[3] = 0;
  }
}
bool oilTempControl(){
  TT447 = get_config_parameter(PARAM_TT447);
  TT444 = get_config_parameter(PARAM_TT444);

  if(get_config_parameter(PARAM_TT447) < 70){
    heater = true;
    set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_DIVERTER_XV);
  }
  else if(get_config_parameter(PARAM_TT447) > 85){
    heater = false;
    set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_DIVERTER_XV);
  }
  if(heater){
    if(get_config_parameter(PARAM_TT444) > 120){set_config_bit(CONFIG_PARAM_RELAYS,FALSE,RELAY_BIT_OIL_HEATER);}
    else if(get_config_parameter(PARAM_TT444) < 70){set_config_bit(CONFIG_PARAM_RELAYS,TRUE,RELAY_BIT_OIL_HEATER);}
  }
}
bool operationCheck(){
  return true;
}


 
 