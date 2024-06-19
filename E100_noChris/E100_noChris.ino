#include "E100.h"

void setup() {
 Serial.begin(9600);
  i2cSetup();
}

void loop() {
  i2cTransceive(dataTime);
  printData(printTime);

  if(!manual){
    oilTempControl();
    waterDropoutControl();

    switch(STATE){
      case R050_OK:
        if(TT447 < 75){
          READYOP1 = false;
          READYOP2 = false;
          VFD1EN = false;
          XVFD1 = false;
        }
        else if(TT447 > 80){
          if(operationCheck()){
            READYOP1 = true;
            READYOP2 = true;
            VFD1EN = READYIP1;
            XVFD1 = READYIP1;
          }
          else{
            READYOP1 = false;
            READYOP2 = false;
            VFD1EN = false;
            XVFD1 = false;
          }
        }
        if(READYIP1 && READYIP2 && READYOP1 && READYOP2){
          STATE = INIT_EVO;
        }
      case INIT_EVO:
        
        if(READYOP1){
          if(PT454 > 15){ EVOEN = true; EVOHZ = 20; }
          if(EVOHZ == 20){
            if(!timer[0]){timer[0] = millis();}
            else if(millis() - timer[0] < 60000*2 && timer[0]){ //decrease ACT1 pos from 75% to 25% over 2min
              mxy = (double)((-1*timer[0]/2400)+75)/100;
              if(mxy >= 0.25){ACT1Pos = ACT1_MAX*mxy;}
            } //hold ACT1 pos for 1min
            else if(60000*3 < millis() - timer[0] < 60000*8){ //increase ACT1 pos from 25% to 75% over 5min
              mxy = (double)((-1*timer[0]/6000)+25)/100;
              if(mxy <= 0.75){ACT1Pos = ACT1_MAX*mxy;}
            }
            else if(millis() - timer[0] > 60000*8){
              STATE = BALANCE_EVO;
            }
          }
        }
      break;

      case BALANCE_EVO:
        if(PT454 > 9){EVOHZ++;}
        else if(PT454 < 8){EVOHZ++;}//maybe use a PID controller here
    }
  }
}

