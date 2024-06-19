void i2cTransceive(int dataTm){
  if(!dataTimer){dataTimer = millis();}
  if(millis() - dataTimer > dataTm){
    for(int i = 0; i < AIsize;i++){
      if(AIdata[i].key.indexOf("TT") && AIdata[i].channel != -1){
        if(AIdata[i].channel == 1 && !bitRead(mcpExist,0)){
          AIdata[i].raw = mcp1.readThermocouple();
        }
        else if(AIdata[i].channel == 2 && !bitRead(mcpExist,1)){
          AIdata[i].raw = mcp2.readThermocouple();
        }
        else if(AIdata[i].channel == 3 && !bitRead(mcpExist,2)){
          AIdata[i].raw = mcp3.readThermocouple();
        }
        else if(AIdata[i].channel == 4 && !bitRead(mcpExist,3)){
          AIdata[i].raw = mcp4.readThermocouple();
        }
        else if(AIdata[i].channel == 5 && !bitRead(mcpExist,4)){
          AIdata[i].raw = mcp5.readThermocouple();
        }
        else if(AIdata[i].channel == 6 && !bitRead(mcpExist,5)){
          AIdata[i].raw = mcp6.readThermocouple();
        }
        else if(AIdata[i].channel == 7 && !bitRead(mcpExist,6)){
          AIdata[i].raw = mcp7.readThermocouple();
        }
        else if(AIdata[i].channel == 8 && !bitRead(mcpExist,7)){
          AIdata[i].raw = mcp8.readThermocouple();
        }
        else{;}
        if(0 < AIdata[i].channel <= 8 && !bitRead(mcpExist,AIdata[i].channel)){
          *AIdata[i].value = AIdata[i].avg.filter(AIdata[i].raw);
        }
      }
      else if(AIdata[i].channel != -1){
        AIdata[i].raw = AIdata[i].adc.read(AIdata[i].channel,SD);
        if(AIdata[i].rating != -1){
          *AIdata[i].value = AIdata[i].avg.filter(map((int)AIdata[i].raw, 820, 4096, 0, AIdata[i].rating));
        }
        else{*AIdata[i].value = AIdata[i].raw;}
      }
    }

    for(int i = 0; i < AOsize;i++){
      if(AOdata[i].channel != -1){
        dac1.setChannelValue((MCP4728_channel_t)(AOdata[i].channel-1), *AOdata[i].value);
      }
    }

    for(int i = 0; i < DIsize;i++){
      if(DIdata[i].addr != -1){
        DIdata[i].gpio.digitalRead(DIdata[i].addr,DIdata[i].value);
      }
    }

    for(int i = 0; i < DOsize;i++){
      if(DOdata[i].addr != -1){
        DOdata[i].gpio.digitalWrite(DOdata[i].addr,*DOdata[i].value);
      }
    }
    dataTimer = 0;
  }
}

void RS485transceive(){
  if(*RS485data[rtuCnt].value != RS485data[rtuCnt].prev && RS485data[rtuCnt].ID != -1){
    uint8_t result = mbRTU.writeSingleRegister(RS485data[rtuCnt].ID,RS485data[rtuCnt].reg,*RS485data[rtuCnt].value);
    RS485data[rtuCnt].prev = *RS485data[rtuCnt].value;
    if (result != mbRTU.ku8MBSuccess){
      Serial.print(RS485data[rtuCnt].name);
      Serial.print(" Connection Error: 0x");
      Serial.println(result,HEX);
    }
  }
  rtuCnt++;
  if(rtuCnt >= RS485size){rtuCnt = 0;}
}


void waterDropoutControl(){
  if(LSH321){if(!debouncer[0]){debouncer[0] = millis();}}
  else{debouncer[0] = 0;}
  if(millis() - debouncer[0] > 300 && debouncer[0]){
    XV323 = true;
    debouncer[0] = 0;
  }
  if(LSL322){if(!debouncer[1]){debouncer[1] = millis();}}
  else{debouncer[1] = 0;}
  if(millis() - debouncer[1] > 300 && debouncer[1]){
    XV323 = false;
    debouncer[1] = 0;
  }
  if(LSH521){if(!debouncer[2]){debouncer[2] = millis();}}
  else{debouncer[2] = 0;}
  if(millis() - debouncer[2] > 300 && debouncer[2]){
    XV523 = true;
    debouncer[2] = 0;
  }
  if(LSL522){if(!debouncer[3]){debouncer[3] = millis();}}
  else{debouncer[3] = 0;}
  if(millis() - debouncer[3] > 300 && debouncer[3]){
    XV523 = false;
    debouncer[3] = 0;
  }
}
bool oilTempControl(){
  if(TT447 < 70){
    heater = true;
    XV109 = false;
  }
  else if(TT447 > 85){
    heater = false;
    XV109 = true;
  }
  if(heater){
    if(TT444 > 120){HTR443 = false;}
    else if(TT444 < 70){HTR443 = true;}
  }
}
bool operationCheck(){
  return true;
}




void i2cSetup(){
  adc1.init();
  while(1){
    if (!dac1.begin()){
      Serial.println("Failed to find MCP4728 chip");
      delay(3000);
    }
    else{break;}
  }
  if(!mcp1.begin(0x60)){
    Serial.println("WARNING!! Couldnt detect mcp1(0x60)");
    bitSet(mcpExist,0);
  }
  else{
    mcp1.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp1.setThermocoupleType(MCP9600_TYPE_K);
    mcp1.enable(true);
  }

  if(!mcp2.begin(0x61)){
    Serial.println("WARNING!! Couldnt detect mcp2(0x61)");
    bitSet(mcpExist,1);
  }
  else{
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp2.setThermocoupleType(MCP9600_TYPE_K);
    mcp2.enable(true);
  }

  if(!mcp3.begin(0x62)){
    Serial.println("WARNING!! Couldnt detect mcp3(0x62)");
    bitSet(mcpExist,2);
  }
  else{
    mcp3.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp3.setThermocoupleType(MCP9600_TYPE_K);
    mcp3.enable(true);
  }

  if(!mcp4.begin(0x63)){
    Serial.println("WARNING!! Couldnt detect mcp4(0x63)");
    bitSet(mcpExist,3);
  }
  else{
    mcp4.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp4.setThermocoupleType(MCP9600_TYPE_K);
    mcp4.enable(true);
  }

  if(!mcp5.begin(0x64)){
    Serial.println("WARNING!! Couldnt detect mcp5(0x64)");
    bitSet(mcpExist,4);
  }
  else{
    mcp5.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp5.setThermocoupleType(MCP9600_TYPE_K);
    mcp5.enable(true);
  }
  
  if(!mcp6.begin(0x65)){
    Serial.println("WARNING!! Couldnt detect mcp6(0x65)");
    bitSet(mcpExist,5);
  }
  else{
    mcp6.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp6.setThermocoupleType(MCP9600_TYPE_K);
    mcp6.enable(true);
  }
  
  if(!mcp7.begin(0x66)){
    Serial.println("WARNING!! Couldnt detect mcp7(0x66)");
    bitSet(mcpExist,6);
  }
  else{
    mcp7.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp7.setThermocoupleType(MCP9600_TYPE_K);
    mcp7.enable(true);
  }

  if(!mcp8.begin(0x67)){
    Serial.println("WARNING!! Couldnt detect mcp8(0x67)");
    bitSet(mcpExist,7);
  }
  else{
    mcp8.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp8.setThermocoupleType(MCP9600_TYPE_K);
    mcp8.enable(true);
  }

  gpio.begin();
  gpio.pinMode(P0_0, OUTPUT);
  gpio.pinMode(P0_1, OUTPUT);
  gpio.pinMode(P0_2, OUTPUT);
  gpio.pinMode(P0_3, OUTPUT);
  gpio.pinMode(P0_4, OUTPUT);
  gpio.pinMode(P0_5, OUTPUT);
  gpio.pinMode(P0_6, OUTPUT);
  gpio.pinMode(P0_7, OUTPUT);
  gpio.pinMode(P1_0, OUTPUT);
  gpio.pinMode(P1_1, OUTPUT);
  gpio.pinMode(P1_2, OUTPUT);
  gpio.pinMode(P1_5, OUTPUT);
  gpio.pinMode(P1_6, OUTPUT);
  gpio.pinMode(P1_7, OUTPUT);
  gpio.pinMode(P2_0, OUTPUT);
  gpio.pinMode(P2_1, OUTPUT);
  gpio.pinMode(P2_2, OUTPUT);
  gpio.pinMode(P2_3, OUTPUT);
  gpio.pinMode(P2_4, INPUT_PULLUP, true);
  gpio.pinMode(P2_5, INPUT_PULLUP, true);
  gpio.pinMode(P2_6, INPUT_PULLUP, true);
  gpio.pinMode(P2_7, INPUT_PULLUP, true);
  gpio.pinMode(P3_0, INPUT_PULLUP, true);
  gpio.pinMode(P3_1, INPUT_PULLUP, true);
  gpio.pinMode(P3_2, INPUT_PULLUP, true);
  gpio.pinMode(P3_3, INPUT_PULLUP, true);
  gpio.pinMode(P3_4, INPUT_PULLUP, true);
  gpio.pinMode(P3_5, INPUT_PULLUP, true);
  gpio.pinMode(P3_6, INPUT_PULLUP, true);
  gpio.pinMode(P3_7, INPUT_PULLUP, true);
  gpio.pinMode(P4_0, INPUT_PULLUP, true);
  gpio.pinMode(P4_1, INPUT_PULLUP, true);
  gpio.digitalWrite(P0_0, LOW);
  gpio.digitalWrite(P0_1, LOW);
  gpio.digitalWrite(P0_2, LOW);
  gpio.digitalWrite(P0_3, LOW);
  gpio.digitalWrite(P0_4, LOW);
  gpio.digitalWrite(P0_5, LOW);
  gpio.digitalWrite(P0_6, LOW);
  gpio.digitalWrite(P0_7, LOW);
  gpio.digitalWrite(P1_0, LOW);
  gpio.digitalWrite(P1_1, LOW);
  gpio.digitalWrite(P1_2, LOW);
  gpio.digitalWrite(P1_5, LOW);
  gpio.digitalWrite(P1_6, LOW);
  gpio.digitalWrite(P1_7, LOW);
  gpio.digitalWrite(P2_0, LOW);
  gpio.digitalWrite(P2_1, LOW);
  gpio.digitalWrite(P2_2, LOW);
  gpio.digitalWrite(P2_3, LOW);


}

void preTransmission() {digitalWrite(RE_DE1, 1);}
void postTransmission() {digitalWrite(RE_DE1, 0);}

void pinModeSetup(){
  pinMode(ESTOP_BREAK, OUTPUT);
  digitalWrite(ESTOP_BREAK, HIGH);  // HIGH to keep Estop loop intact
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);  // HIGH to provide power to Daughterboard
  pinMode(TRACO_24VDC, OUTPUT);
  digitalWrite(TRACO_24VDC, HIGH);  // HIGH to enable 24V Power Supply
}
 