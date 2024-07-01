

/*
    Author: Christopher Glanville
    Description:    IO class to handle all external connections (Both analog and digital) to and from the Giga. Debounce functions 
                    for button inputs are also handled here. 
   Date: 26/5/24
   updated : 26/5/24
*/

#include <PI4IOE5V6534Q.h>
#include <Arduino.h>
#include <ADS7828.h>
#include <MCP4728.h>
#include <Wire.h>
#include <stdint.h>
#include <movingAvg.h>
#include <Ewma.h>
#include "Adafruit_MCP9600.h"
#include "config_e100.h"
#include "io_e100.h"
#include "ble_e100.h"  // Required to update the name of the C200 PCB in the event of a change to SW1.3

// ************ Pin definitions *****************

// ************ Pin definitions *****************
#if defined(HARDWARE_1_2)

//EXP 1 (ADS-7828)
#define EVO_DISCHARGE_PRESSURE                  0            // TB44, ADS-7828 Addr 0x48, channel 0
#define EVO_DISCHARGE_PRESSURE_EXP              PT_Exp1      // Connect each parameter to the correct I2C Expander
#define PSA_PRESS_1                             1            // TB45, ADS-7828 Addr 0x48, channel 1
#define PSA_PRESS_1_EXP                         PT_Exp1      
#define PSA_PRESS_2                             2            // TB46, ADS-7828 Addr 0x48, channel 2
#define PSA_PRESS_2_EXP                         PT_Exp1    
#define FINAL_DISCHARGE_PRESSURE                3            // TB47, ADS-7828 Addr 0x48 channel 3
#define FINAL_DISCHARGE_PRESSURE_EXP            PT_Exp1
#define ACT_1_POSITION                          4            // TB48, ADS-7828 Addr 0x48, channel 4
#define ACT_1_POSITION_EXP                      PT_Exp1                 
#define ACT_2_POSITION                          5            // TB49 ADS-7828 Addr 0x48 channel 5
#define ACT_2_POSITION_EXP                      PT_Exp1                   
#define PSA_1_FLOW                              6            // TB450 ADS-7828 Addr 0x48 channel 6
#define PSA_1_FLOW_EXP                          PT_Exp1                      
#define GAS_SUCTION_PRESSURE                    7            // TB51 ADS-7828 Addr 0x48 channel 7
#define GAS_SUCTION_PRESSURE_EXP                PT_Exp1            


//EXP 2 (PI4IOE5V6534Q)
#define LOCAL_INT                               P0_0        // R24.1
#define LOCAL_INT_EXP                           IO_Exp2                   
#define REMOTE_RESET                            P0_1        // R24.2
#define REMOTE_RESET_EXP                        IO_Exp2    
#define GREEN_PILOT                             P0_2        // R24.3
#define GREEN_PILOT_EXP                         IO_Exp2  
#define AMBER_PILOT                             P0_3        // R24.4
#define AMBER_PILOT_EXP                         IO_Exp2    
#define RED_PILOT                               P0_4        // R24.5
#define RED_PILOT_EXP                           IO_Exp2         

#define OIL_HTR                                 P0_5        // R24.6
#define OIL_HTR_EXP                             IO_Exp2    
#define COOLANT_FAN_RLY                         P0_6        // R24.7
#define COOLANT_FAN_RLY_EXP                     IO_Exp2    
#define COOLANT_PUMP_ON                         P0_7
#define COOLANT_PUMP_ON_EXP                     IO_Exp2     // R24.8
#define DIVERTER_XV_TO_HX101B                   P1_0   
#define DIVERTER_XV_TO_HX101B_EXP               IO_Exp2     // R24.9
#define VE311_DROPOUT                           P1_1
#define VE311_DROPOUT_EXP                       IO_Exp2     // R24.10

#define VE511_DROPOUT                           P1_2        // R24.11
#define VE511_DROPOUT_EXP                       IO_Exp2                
#define XV_RPSA1_GAS_FEED                       P1_3        // R24.12
#define XV_RPSA1_GAS_EXP                        IO_Exp2                 
#define IP_XV_RPSA2_GAS_FEED                    P1_4        // R24.13
#define IP_XV_RPSA2_GAS_EXP                     IO_Exp2               
#define IP_XV_RPSA1_VENT_GAS                    P1_5        // R24.14
#define IP_XV_RPSA1_VENT_EXP2                   IO_Exp2           
#define IP_XV_RPSA2_VENT_GAS                    P1_6        // R24.15
#define IP_XV_RPSA2_VENT_EXP2                   IO_Exp2            

#define READY_R050_1                            P1_7         // R24.16
#define READY_R050_1_EXP                        IO_Exp2                 
#define READY_R050_2                            P2_0         // R24.17
#define READY_R050_2_EXP                        IO_Exp2                 
#define RPSA1_VFD_RUN                           P2_1         // R24.18
#define RPSA1_VFD_RUN_EXP                       IO_Exp2                         
#define RPSA2_VFD_RUN                           P2_2         // R24.19
#define RPSA2_VFD_RUN_EXP                       IO_Exp2                
#define EVO_VFD_RUN                             P2_3         // R24.20
#define EVO_VFD_RUN_EXP                         IO_Exp2                  
#define GLOB_ITLCK_IP                           P2_4         // R24.0
#define GLOB_ITLCK_IP_EXP                       IO_Exp2

#define LOCAL_ESTOP_IP                          P2_5         // TB 5.2
#define LOCAL_ESTOP_IP_EXP                      IO_Exp2              
#define GREEEN_BUTTON_IP                        P2_6         // TB 6.4
#define GREEEN_BUTTON_IP_EXP                    IO_Exp2          
#define RED_BUTTON_IP                           P2_7         // TB 8.4
#define RED_BUTTON_IP_EXP                       IO_Exp2  
#define LSR_IP                                  P3_0         // TB 12.10
#define LSR_IP_EXP                              IO_Exp2  
#define COOLANT_FLOW_SWITCHES_IP_1              P3_1         // TB 13.4
#define COOLANT_FLOW_SWITCHES_IP_1_EXP          IO_Exp2  

#define HIGH_LEVEL_DROPOUT_SW_321               P3_2          // R24.22
#define HIGH_LEVEL_DROPOUT_SW_321_EXP           IO_Exp2
#define LOW_LEVEL_DROPOUT_SW322                 P3_3          // R24.23
#define LOW_LEVEL_DROPOUT_SW322_EXP             IO_Exp2                  
#define HIGH_LEVEL_DROPOUT_SW521                P3_4          // R24.24
#define HIGH_LEVEL_DROPOUT_SW521_EXP            IO_Exp2       
#define LOW_LEVEL_DROPOUT_SW522                 P3_5          // R24.25
#define LOW_LEVEL_DROPOUT_SW522_EXP             IO_Exp2 
#define READYIP_R050_1                          P3_6          // R24.26
#define READYIP_R050_1_EXP                      IO_Exp2
#define READYIP_R050_2                          P3_7          // R24.27
#define READYIP_R050_2_EXP                      IO_Exp2 

//EXP3 (MCP4728T)
#endif

// MCP9600 I2C address
#define MCP_ADDRESS1                            0x60
#define MCP_ADDRESS2                            0x61
#define MCP_ADDRESS3                            0x62
#define MCP_ADDRESS4                            0x63
#define MCP_ADDRESS5                            0x64
#define MCP_ADDRESS6                            0x65
#define MCP_ADDRESS7                            0x66
#define MCP_ADDRESS8                            0x67

#define EXP1_IO_ADDR                            0x48         // ADS7828 i2c address
#define EXP2_IO_ADDR                            0x22         // PI4IOE5V6534Q address

#define DEBOUNCE_TIMER 1000

#define TT_SIZE 6

// uint8_t red_button = false;
// uint8_t amber_button = false;
// uint8_t green_button = false;
// uint8_t estop_button = false;
// //uint8_t hydraulicFluidLvl = false;
// uint8_t coolantFlowSwitch = false;
// //uint8_t hydFilterSwitch = false;
// uint8_t globalInterlock = false;
// uint8_t schmersal = false;

// bool PressgreenButton = false;
// bool pauseButton = false;
// bool OldValue_Redbutton = false, OldValue_greenbutton = false, OldValue_amberbutton = false, OldValue_CoolantFlowSwitch = false, OldValue_estop = false;
// int tracker = 0, trackerRed = 0, trackerGreen = 0, trackerAmber = 0, trackerCoolantFlowSwitch = 0, estop_tracker = 0;
// bool temp_Redbutton, temp_yellowbutton, temp_greenbutton, temp_Flow, temp_CoolantFlowSwitch, temp_estop;

// unsigned long onOffButton = false;

// bool lastValue = false;
// bool OldValue_Flow = false;
// unsigned long debouncetime = 0, hydraulicPumpTwoStartupTimer = 0;
// float coolantTemp;
// float coolantTemp_OLD;

// float s1_suction_tank_pt, s3_discharge_pt;
// float hydraulic_oil_tt;
// float s1DischargeTemp_TT809;

//ADS7828 EXP1 Channels
Ewma avgPT_CH0(0.02);
Ewma avgPT_CH1(0.02);
Ewma avgPT_CH2(0.02);
Ewma avgPT_CH3(0.02);
Ewma avgPT_CH4(0.02);
Ewma avgPT_CH5(0.02);
Ewma avgPT_CH6(0.02);
Ewma avgPT_CH7(0.02);




#if defined(HARDWARE_1_2)
  // i2c, exp1, exp2, exp3 instance
  TwoWire i2c(20, 21);
  PI4IOE5V6534Q u65_io(EXP2_IO_ADDR, i2c);    // IO EXPANDER
  ADS7828 PT_Exp1;                            //ADC
  MCP4728 dac_Exp3;                           //DAC
  Adafruit_MCP9600 mcp1, mcp2, mcp3, mcp4, mcp5, mcp6, mcp7, mcp8; //Thermocouple (TC1, TC2, TC3, TC4, TC5, TC6, TC7, TC8)
  uint8_t mcpExist = 0;
  int errCnt = 0;
#endif

/*
    Name:         float_map
    Arguments:    float original_value    The original value to be rescaled
                  float in_min            The origianl scale we are moving away from (lower limit)
                  float in_max            The origianl scale we are moving away from (upper limit)
                  float out_min           The new scale we are translating into (lower limit)
                  float out_max           The new scale we are translating into (upper limit)
    Returns:      float                   \The new value translated onto the new scale as a float
    Description:  The map function provided by arduino only translates long integers. We need to scale some of the 
                  PT values to decimal values with more accuracy and so a float equivilent was required. 
*/
float float_map(float original_value, float in_min, float in_max, float out_min, float out_max)
{
  return (original_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
    Name:         io_setup
    Arguments:    void
    Returns:      nil
    Description:  io_setup sets all of the initial conditions for MCP9600.
                  ADS7828 (EXP1) and PI4IOE5V6534Q (EXP2) init are done in E100.ino              
*/

// setup for MCP9600
void io_setup() {

  // Initialize the I2C Communications
  i2c.setClock(400000UL);   // clock speed 400kHz
  Wire1.setClock(400000UL);
  Wire.begin();       // EXP1 and EXP2 i2c
  Wire1.begin();
  Wire1.beginTransmission(0x60);

  dac_Exp3.attach(Wire1, -1); /*Use -1 to indicate LDAC is grounded and not controlled, it is connected to 0VDC 
                                   the DAC outputs will update immediately whenever new data is written to its input registers*/

  u65_io.begin();
  u65_io.pinMode(P0_4, OUTPUT);   // Red pilot relay, R24.5
  u65_io.pinMode(P0_3, OUTPUT);   // Amber pilot relay, R24.4
  u65_io.pinMode(P0_2, OUTPUT);   // Green pilot relay, R24.3
  u65_io.pinMode(P0_5, OUTPUT);   // Oil heater
  u65_io.pinMode(P0_6, OUTPUT);   // Coolant fans
  u65_io.pinMode(P0_7, OUTPUT);   // Coolant pumps
  u65_io.pinMode(P1_0, OUTPUT);   // Diverter XV to HX101B
  u65_io.pinMode(P1_1, OUTPUT);   // VE311
  u65_io.pinMode(P1_2, OUTPUT);   // VE511
  u65_io.pinMode(P1_3, OUTPUT);   // XV RPSA 1 - Gas feed R24.12
  u65_io.pinMode(P1_4, OUTPUT);   // XV RPSA 2 - Gas feed R24.13
  u65_io.pinMode(P1_5, OUTPUT);   // XV RPSA 1 - Vent Gas R24.14
  u65_io.pinMode(P1_6, OUTPUT);   // XV RPSA 2 - Vent Gas R24.15
  u65_io.pinMode(P1_7, OUTPUT);   // Read OP 050 1 - R24.16
  u65_io.pinMode(P2_0, OUTPUT);   // Read OP 050 2 - R24.17
  u65_io.pinMode(P2_1, OUTPUT);   // RPSA.1 VFD RUN R24.18
  u65_io.pinMode(P2_2, OUTPUT);   // RPSA.2 VFD RUN R24.19
  u65_io.pinMode(P2_3, OUTPUT);   // EVO VFD RUN R24.20
  u65_io.pinMode(P2_4, INPUT_PULLUP, true);    // Global Interlock - R24.0 NO1 
  u65_io.pinMode(P2_5, INPUT_PULLUP, true);    // Local E-STOP - TB5.2
  u65_io.pinMode(P2_6, INPUT_PULLUP, true);    // Coolant flow switch TB13.4
  u65_io.pinMode(P2_7, INPUT_PULLUP, true);    // Red button switch input TB8.4
  u65_io.pinMode(P3_0, INPUT_PULLUP, true);    // LSR IP TB12.10
  u65_io.pinMode(P3_1, INPUT_PULLUP, true);    // Coolant flow switch TB13.4
  u65_io.pinMode(P3_2, INPUT_PULLUP, true);    // High level dropout switch - R24.22
  u65_io.pinMode(P3_3, INPUT_PULLUP, true);    // Low level dropout switch - R24.23
  u65_io.pinMode(P3_4, INPUT_PULLUP, true);    // High level dropout switch - R24.24
  u65_io.pinMode(P3_5, INPUT_PULLUP, true);    // Low level dropout switch - R24.25
  u65_io.pinMode(P3_6, INPUT_PULLUP, true);    // Ready IP R050 1 - R24.26
  u65_io.pinMode(P3_7, INPUT_PULLUP, true);    // Ready IP R050 2 - R24.27

  //Initialize ADS7828 (Exp1) 
  PT_Exp1.begin(EXP1_IO_ADDR, &i2c);

//MCP9600  Thermocouple init
#if defined(HARDWARE_1_2)
  if (!mcp1.begin(MCP_ADDRESS1)) {                              //TB32
    Serial.println("WARNING!! Couldnt detect mcp1(0x60)");
    bitSet(mcpExist, 0);
  } else {
    Serial.println("mcp1(0x60) initialized!");
    mcp1.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp1.setThermocoupleType(MCP9600_TYPE_K);
    mcp1.enable(true);
  }

  if (!mcp2.begin(MCP_ADDRESS2)) {                                    //TB33
    Serial.println("WARNING!! Couldnt detect mcp2(0x61)");
    bitSet(mcpExist, 1);
  } else {
    Serial.println("mcp2(0x61) initialized!");
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp2.setThermocoupleType(MCP9600_TYPE_K);
    mcp2.enable(true);
  }

  if (!mcp3.begin(MCP_ADDRESS3)) {                                    //TB34
    Serial.println("WARNING!! Couldnt detect mcp3(0x62)");
    bitSet(mcpExist, 2);
  } else {
    Serial.println("mcp3(0x62) initialized!");
    mcp3.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp3.setThermocoupleType(MCP9600_TYPE_K);
    mcp3.enable(true);
  }

  if (!mcp4.begin(MCP_ADDRESS4)) {                                     //TB35
    Serial.println("WARNING!! Couldnt detect mcp4(0x63)");
    bitSet(mcpExist, 3);
  } else {
    Serial.println("mcp4(0x63) initialized!");
    mcp4.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp4.setThermocoupleType(MCP9600_TYPE_K);
    mcp4.enable(true);
  }

  if (!mcp5.begin(MCP_ADDRESS5)) {                                     //TB36
    Serial.println("WARNING!! Couldnt detect mcp5(0x64)");
    bitSet(mcpExist, 4);
  } else {
    Serial.println("mcp5(0x64) initialized!");
    mcp5.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp5.setThermocoupleType(MCP9600_TYPE_K);
    mcp5.enable(true);
  }

  if (!mcp6.begin(MCP_ADDRESS6)) {                                     //TB37
    Serial.println("WARNING!! Couldnt detect mcp6(0x65)");
    bitSet(mcpExist, 5);
  } else {
    Serial.println("mcp6(0x65) initialized!");
    mcp6.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp6.setThermocoupleType(MCP9600_TYPE_K);
    mcp6.enable(true);
  }
  if (!mcp7.begin(MCP_ADDRESS7)) {                                     //TB38
    Serial.println("WARNING!! Couldnt detect mcp7(0x66)");
    bitSet(mcpExist, 6);
  } else {
    Serial.println("mcp7(0x66) initialized!");
    mcp7.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp7.setThermocoupleType(MCP9600_TYPE_K);
    mcp7.enable(true);
  }

    if (!mcp8.begin(MCP_ADDRESS8)) {                                   //TB39
    Serial.println("WARNING!! Couldnt detect mcp8(0x67)");
    bitSet(mcpExist, 7);
  } else {
    Serial.println("mcp8(0x67) initialized!");
    mcp8.setADCresolution(MCP9600_ADCRESOLUTION_12);
    mcp8.setThermocoupleType(MCP9600_TYPE_K);
    mcp8.enable(true);
  }
  //MCP4728 init
  dac_Exp3.readRegisters();
  dac_Exp3.selectVref(MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD, MCP4728::VREF::VDD);
  dac_Exp3.selectPowerDown(MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM, MCP4728::PWR_DOWN::GND_100KOHM);
  dac_Exp3.selectGain(MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1, MCP4728::GAIN::X1);
  dac_Exp3.analogWrite(MCP4728::DAC_CH::A, 4095);    // 111 = 0.261 - TB40
  dac_Exp3.analogWrite(MCP4728::DAC_CH::B, 2048);    // 222 = 0.529 - TB41
  dac_Exp3.analogWrite(MCP4728::DAC_CH::C, 1024);    // TB42
  dac_Exp3.analogWrite(MCP4728::DAC_CH::D, 0);    // TB43
  dac_Exp3.enable(true);
  dac_Exp3.readRegisters();
  
#endif
  Serial.println("IO setup complete");
}

void digital_io_loop()
{
  uint8_t di_temp_val; 

  // *************** DIGITAL OUTPUTS ****************************
  // Green pilot light
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_GREEN_PILOT)){
    u65_io.digitalWrite(P0_2, HIGH);
  }else{
    u65_io.digitalWrite(P0_2, LOW);
  }

  // Amber pilot light
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_AMBER_PILOT)){
    u65_io.digitalWrite(P0_3, HIGH);
  }else{
    u65_io.digitalWrite(P0_3, LOW);
  }

  // Red pilot light
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_RED_PILOT)){
    u65_io.digitalWrite(P0_4, HIGH);
  }else{
    u65_io.digitalWrite(P0_4, LOW);
  }

  // Oil heater R24.6 
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_OIL_HEATER)){
    u65_io.digitalWrite(P0_5, HIGH);
  }else{
    u65_io.digitalWrite(P0_5, LOW);
  }

  // Coolant fans - R24.7 P0_6
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_COOLANT_FANS)){
    u65_io.digitalWrite(P0_6, HIGH);
  }else{
    u65_io.digitalWrite(P0_6, LOW);
  }

  // Coolant pump - R24.8 P0_7
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_COOLANT_PUMP)){
    u65_io.digitalWrite(P0_7, HIGH);
  }else{
    u65_io.digitalWrite(P0_7, LOW);
  }

  // Remote reset - R24.2/P0_1
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_REMOTE_RESET)){
    u65_io.digitalWrite(P0_1, HIGH);
  }else{
    u65_io.digitalWrite(P0_1, LOW);
  }

  // Local interlock - R24.1/P0_0
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_LOCAL_INTERLOCK)){
    u65_io.digitalWrite(P0_0, HIGH);
  }else{
    u65_io.digitalWrite(P0_0, LOW);
  }

  // Diversion XV to HX101B - R24.9
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_DIVERTER_XV)){
    u65_io.digitalWrite(P1_0, HIGH);
  }else{
    u65_io.digitalWrite(P1_0, LOW);
  }
  
  // VE311 Drop-out XV R24.10
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_VE311)){
    u65_io.digitalWrite(P1_1, HIGH);
  }else{
    u65_io.digitalWrite(P1_1, LOW);
  }

  // VE511 Drop-out XV R24.11
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_VE511)){
    u65_io.digitalWrite(P1_2, HIGH);
  }else{
    u65_io.digitalWrite(P1_2, LOW);
  }

  //  XV RPSA.1 Gas feed R24.12
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_XV_RPSA1_GAS_FEED)){
    u65_io.digitalWrite(P1_3, HIGH);
  }else{
    u65_io.digitalWrite(P1_3, LOW);
  }

  //  XV RPSA.2 Gas feed R24.13
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_XV_RPSA2_GAS_FEED)){
    u65_io.digitalWrite(P1_4, HIGH);
  }else{
    u65_io.digitalWrite(P1_4, LOW);
  }

  //  XV RPSA.1 Vent gas R24.14
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_XV_RPSA1_VENT_GAS)){
    u65_io.digitalWrite(P1_5, HIGH);
  }else{
    u65_io.digitalWrite(P1_5, LOW);
  }

  //  XV RPSA.2 Vent gas R24.15
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_XV_RPSA2_VENT_GAS)){
    u65_io.digitalWrite(P1_6, HIGH);
  }else{
    u65_io.digitalWrite(P1_6, LOW);
  }

  //  XV RPSA.2 Vent gas R24.16
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_READY_R050_1)){
    u65_io.digitalWrite(P1_7, HIGH);
  }else{
    u65_io.digitalWrite(P1_7, LOW);
  }
  
  //  R24.17
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_READY_R050_2)){
    u65_io.digitalWrite(P2_0, HIGH);
  }else{
    u65_io.digitalWrite(P2_0, LOW);
  }

  //  R24.18
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_PSA1_VFD_RUN)){
    u65_io.digitalWrite(P2_1, HIGH);
  }else{
    u65_io.digitalWrite(P2_1, LOW);
  }

  //  R24.19
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_PSA2_VFD_RUN)){
    u65_io.digitalWrite(P2_2, HIGH);
  }else{
    u65_io.digitalWrite(P2_2, LOW);
  }

  //  R24.20
  if (test_config_parameter(CONFIG_PARAM_RELAYS, RELAY_BIT_EVO_VFD_RUN)){
    u65_io.digitalWrite(P2_3, HIGH);
  }else{
    u65_io.digitalWrite(P2_3, LOW);
  }

  // **************** Digital Inputs ****** BEGIN *******************************
  u65_io.digitalRead(P2_4, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_GLOBAL_INT);

  u65_io.digitalRead(P2_5, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_LOCAL_ESTOP);

  u65_io.digitalRead(P2_6, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  // If EITHER of the digital input OR the modbus remote green button press are active, then....
  if (di_temp_val || test_config_parameter(CONFIG_PARAM_OP_STATE_ALL, OP_STATE_GREEN_BUTTON)){
    set_config_parameter(CONFIG_PARAM_INPUTS, true, INPUT_BIT_GREEN_BUTTON);
  }else{
    set_config_parameter(CONFIG_PARAM_INPUTS, false, INPUT_BIT_GREEN_BUTTON);
  }

  u65_io.digitalRead(P2_7, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  // If EITHER of the digital input OR the modbus remote red button press are active, then....
  if (di_temp_val || test_config_parameter(CONFIG_PARAM_OP_STATE_ALL, OP_STATE_RED_BUTTON)){
    Serial.println("Button press");
    set_config_parameter(CONFIG_PARAM_INPUTS, true, INPUT_BIT_RED_BUTTON);
  }else{
    set_config_parameter(CONFIG_PARAM_INPUTS, false, INPUT_BIT_RED_BUTTON);
  }
  
  u65_io.digitalRead(P3_0, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_LSR_IP);

  u65_io.digitalRead(P3_1, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_COOLANT_FS);

  u65_io.digitalRead(P3_2, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_HIGH_LEVEL_DO_1);

  u65_io.digitalRead(P3_3, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_LOW_LEVEL_DO_1);

  u65_io.digitalRead(P3_4, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_HIGH_LEVEL_DO_2);

  u65_io.digitalRead(P3_5, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_LOW_LEVEL_DO_2);

  u65_io.digitalRead(P3_6, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_READY_IP_R050_1);

  u65_io.digitalRead(P3_7, &di_temp_val); // Why can't we just return the value directly like a normal piece of code?!
  set_config_parameter(CONFIG_PARAM_INPUTS, di_temp_val, INPUT_BIT_READY_IP_R050_2);
  // **************** Digital Inputs ****** END *******************************
}

void analog_io_loop(){
  //EVO GAS DISCHARGE PRESSURE
  // Mapping 250psi sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_EVO_GAS_PX, map(avgPT_CH0.filter(EVO_DISCHARGE_PRESSURE_EXP.read(EVO_DISCHARGE_PRESSURE)), 819, 4095, 0, 250));

  //PSA PRESS 1 - TB45 - Mapping 250psi sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_PSA_PRESS_1_PX, map(avgPT_CH1.filter(PSA_PRESS_1_EXP.read(PSA_PRESS_1)), 819, 4095, 0, 250));

  //PSA PRESS 2 - TB46 Mapping 250psi sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_PSA_PRESS_2_PX, map(avgPT_CH2.filter(PSA_PRESS_2_EXP.read(PSA_PRESS_2)), 819, 4095, 0, 250));

  //FINAL GAS DISCHARGE PRESSURE - TB47 -  Mapping 250psi sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_FINAL_GAS_DISCHARGE_PX, map(avgPT_CH3.filter(FINAL_DISCHARGE_PRESSURE_EXP.read(FINAL_DISCHARGE_PRESSURE)), 819, 4095, 0, 250));

  //ACT 1 POSITION - TB48 Mapping 0-100% sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_ACT_1_PX, map(avgPT_CH4.filter(ACT_1_POSITION_EXP.read(ACT_1_POSITION)), 0, 4095, 0, 100));

  //ACT 2 POSITION - TB49 Mapping 0-100% sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_ACT_2_PX, map(avgPT_CH5.filter(ACT_2_POSITION_EXP.read(ACT_2_POSITION)), 0, 4095, 0, 100));

  //PSA 1 FLOW - TB50
  set_config_parameter(CONFIG_PARAM_PSA_1_PX, avgPT_CH6.filter(PSA_1_FLOW_EXP.read(PSA_1_FLOW)));

  //GAS SUCTION PRESSURE - TB51 - Mapping 50psi sensor provided by Gary Lai in email on 21/6/2024
  set_config_parameter(CONFIG_PARAM_GAS_SUCTION_PX, float_map(avgPT_CH7.filter(GAS_SUCTION_PRESSURE_EXP.read(GAS_SUCTION_PRESSURE)), 819, 4095, 0, 50));

  // TC105 - Coolant temp (Gas) - TB32
  // if (mcp1.getStatus() & MCP960X_STATUS_INPUTRANGE){ // To be used after resistors are installed - See section 6.3.3 of datasheet
  set_config_parameter(CONFIG_PARAM_TC105, mcp1.readThermocouple());

  // TC205 - Coolant temp (Oil) - TB33
  set_config_parameter(CONFIG_PARAM_TC205, mcp2.readThermocouple());
  
  // TC313 - Gas suction temp - TB34
  set_config_parameter(CONFIG_PARAM_TC313, mcp3.readThermocouple());

  // TC444 - Oil heater temp - TB35
  set_config_parameter(CONFIG_PARAM_TC444, mcp4.readThermocouple());
  
  // TC447 - Oil sump temp - TB36
  set_config_parameter(CONFIG_PARAM_TC447, mcp5.readThermocouple());

  // TC.RPSA1 - TB37
  set_config_parameter(CONFIG_PARAM_TC_RPSA1, mcp6.readThermocouple());

  // TC.RPSA2 - TB38
  set_config_parameter(CONFIG_PARAM_TC_RPSA2, mcp7.readThermocouple());

  // TC.SPARE - TB39
  set_config_parameter(CONFIG_PARAM_TC_SPARE, mcp8.readThermocouple());


  //*************** ANALOG OUTPUTS ****************************

  // The DAC - EXP3 is on the different I2C bus SCL1
  dac_Exp3.analogWrite(MCP4728::DAC_CH::A, map(get_config_parameter(CONFIG_PARAM_AO_1, AS_INT),0,100,0,4095));
  dac_Exp3.analogWrite(MCP4728::DAC_CH::B, map(get_config_parameter(CONFIG_PARAM_AO_2, AS_INT),0,100,0,4095));    
  dac_Exp3.analogWrite(MCP4728::DAC_CH::C, map(get_config_parameter(CONFIG_PARAM_AO_3, AS_INT),0,100,0,4095));  
  dac_Exp3.analogWrite(MCP4728::DAC_CH::D, map(get_config_parameter(CONFIG_PARAM_AO_4, AS_INT),0,100,0,4095));
}
