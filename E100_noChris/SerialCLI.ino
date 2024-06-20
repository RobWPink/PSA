#define SPTR_SIZE   20
void SerialCLI() {
  String str;
  char *argBuf[20] = { 0 };
  double argVal = 0;
  while (Serial.available()) { str = Serial.readStringUntil('\n'); }

  if (str.length() > 0) {
    
    //Parse string function wasn't working on GIGA
    char s[100] = { 0 };
    int numArgs = 0;
    int size = sizeof(argBuf);
    strcpy(s, str.c_str());
    argBuf[0] = strtok(s, " ");
    //Serial.print(argBuf[0]);
    for (numArgs = 1; NULL != (argBuf[numArgs] = strtok(NULL, " ")); numArgs++) {
      //Serial.println(argBuf[numArgs] );
      if (size == numArgs) { break; }
    }
    
    for (int n = 0; n < numArgs; n++) {
      String argStr = argBuf[n];
      int digital = false;
      for(int i = 0; i < DOsize; i++){
        if(argStr.equalsIgnoreCase(DOdata[i].key)){
          String argStrVal = argBuf[++n];
          if(argStrVal.equalsIgnoreCase("false")){
            *DOdata[i].value = false;
          }
          else if(argStrVal.equalsIgnoreCase("true")){
            *DOdata[i].value = true;
          }
          else{ *DOdata[i].value = !*DOdata[i].value; }
          digital = true;
          break;
        }
      }
      for(int i = 0; i < AOsize; i++){
        if(argStr.equalsIgnoreCase(AOdata[i].key)){
          String argStrVal = argBuf[++n];
          argVal = argStrVal.toDouble();
          *AOdata[i].value = argVal;
          digital = true;
          break;
        }
      }
      for(int i = 0; i < RS485size; i++){
        if(argStr.equalsIgnoreCase(RS485data[i].key)){
          String argStrVal = argBuf[++n];
          argVal = argStrVal.toDouble();
          *RS485data[i].value = argVal;
          digital = true;
          break;
        }
      }
      if(digital){;}

      else if(argStr.equalsIgnoreCase("ao")) {
        tog[0] = !tog[0];
      }
      else if(argStr.equalsIgnoreCase("ai")) {
        tog[1] = !tog[1];
      }
      else if(argStr.equalsIgnoreCase("do")) {
        tog[2] = !tog[2];
      }
      else if(argStr.equalsIgnoreCase("di")) {
        tog[3] = !tog[3];
      }
      else if(argStr.equalsIgnoreCase("rs485")) {
        tog[4] = !tog[4];
      }
      else if(argStr.equalsIgnoreCase("err")) {
        tog[5] = !tog[5];
      }

      else if(argStr.equalsIgnoreCase("manual")) {
        manual = !manual;
      }
      else if(argStr.equalsIgnoreCase("printTime")) {
        String argStrVal = argBuf[++n];
        printTime = argStrVal.toDouble();
      }
      else if(argStr.equalsIgnoreCase("dataTime")) {
        String argStrVal = argBuf[++n];
        dataTime = argStrVal.toDouble();
      }
      
      else{
        Serial.println("Invalid Entry, type \"help\" or \"h\" for a list of commands.");
      }
    }
  }
}

void printData(int inter){
  if(!printTimer){printTimer = millis();}
  if(millis() - printTimer > inter && printTimer){
    if(tog[0]){
      for(int i = 0; i < AOsize;i++){
        Serial.print(AOdata[i].name);
        Serial.print(": ");
        Serial.println(*AOdata[i].value);
      }
    }

    if(tog[1]){
      for(int i = 0; i < AIsize;i++){
        Serial.print(AIdata[i].name);
        Serial.print(": ");
        Serial.println(*AIdata[i].value);
      }
    }

    if(tog[2]){
      for(int i = 0; i < DOsize;i++){
        Serial.print(DOdata[i].name);
        Serial.print(": ");
        Serial.println(*DOdata[i].value);
      }
    }

    if(tog[3]){
      for(int i = 0; i < DIsize;i++){
        Serial.print(DIdata[i].name);
        Serial.print(": ");
        Serial.println(*DIdata[i].value);
      }
    }

    if(tog[4]){
      for(int i = 0; i < RS485size;i++){
        Serial.print(RS485data[i].name);
        Serial.print(": ");
        Serial.println(*RS485data[i].value);
      }
    }
  }
}