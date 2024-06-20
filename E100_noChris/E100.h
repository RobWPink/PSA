#include <Wire.h>
#include <SPI.h>

#include <PI4IOE5V6534Q.h>
#include <ADS7828_rob.h>
#include "Adafruit_MCP9600.h"
#include <Adafruit_MCP4728.h>
#include <Ewma.h>
#include <ModbusMaster_oneH2.h>

#define MOVING_AVG_SIZE 0.01
#define EVO_SLAVE_ID 1
#define PSA1_SLAVE_ID 3
#define PSA2_SLAVE_ID 4
#define RE_DE1 12
#define ESTOP_BREAK 40
#define LED_PWR 22
#define TRACO_24VDC 23

#define speedAddr_old 0x091A
#define speedAddr 0x2001
#define speedFbAddr 0x2103
#define runAddr_old 0x091B
#define runAddr 0x2000
#define runFbAddr 0x2102
ModbusMaster mbRTU;

ADS7828 adc1(0x48);

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;
Adafruit_MCP9600 mcp3;
Adafruit_MCP9600 mcp4;
Adafruit_MCP9600 mcp5;
Adafruit_MCP9600 mcp6;
Adafruit_MCP9600 mcp7;
Adafruit_MCP9600 mcp8;

Adafruit_MCP4728 dac1;

TwoWire i2c(20, 21);

PI4IOE5V6534Q gpio(0x22, i2c);

enum stt{
  R050_OK,
  INIT_EVO,
  BALANCE_EVO
}STATE;

unsigned long timer[5] = {0};
unsigned long debouncer[4] = {0};
unsigned long printTimer = 0;
bool tog[6] = {false};
unsigned long dataTimer = 0;
bool heater = false;
bool manual = true;
double printTime = 100;
double dataTime = 250;
int ACT1_MAX = 4096;
int rtuCnt = 0;
double TT105  = 0;
double TT205  = 0;
double TT313  = 0;
double TT444  = 0;
double TT447  = 0;
double TT1  = 0;
double TT2  = 0;
double TT3  = 0;
double EVOHZfb = 0;
double VFD2HZfb = 0;
double VFD1HZfb = 0;
double PTevoDischarge = 0;
double PTPSA1 = 0;
double PTPSA2 = 0;
double PTFinalDischarge = 0;
double ACT1Pos = 0;
double ACT2Pos = 0;
double PSAflow = 0;
double PT454 = 0;
double ACT1 = 0;
double ACT2 = 0;

double SPARE = 0;
double VFD1HZ = 0;
double VFD2HZ = 0;
double VFD1EN = 0;
double VFD2EN = 0;
double EVOHZ = 0;
double EVOEN = 0;

uint8_t LSR = 0;
uint8_t LSRRST = 0;
uint8_t GPL = 0;
uint8_t APL = 0;
uint8_t RPL = 0;
uint8_t HTR443 = 0;
uint8_t CLTFAN = 0;
uint8_t CLTPMP = 0;
uint8_t XV109 = 0;
uint8_t XV323 = 0;
uint8_t XV523 = 0;
uint8_t XVFD1 = 0;
uint8_t XVFD2 = 0;
uint8_t XVVNT1 = 0;
uint8_t XVVNT2 = 0;
uint8_t READYOP1 = 0;
uint8_t READYOP2 = 0;
// uint8_t VFD1EN = 0;
// uint8_t VFD2EN = 0;
// uint8_t EVOVFDEN = 0;
uint8_t GSR = 0;
uint8_t ESTOP = 0;
uint8_t GBN = 0;
uint8_t RBN = 0;
uint8_t LSROUT = 0;
uint8_t CLTFLWSW = 0;
uint8_t LSH321 = 0;
uint8_t LSL322 = 0;
uint8_t LSH521 = 0;
uint8_t LSL522 = 0;
uint8_t READYIP1 = 0;
uint8_t READYIP2 = 0;

Ewma avgTT105(MOVING_AVG_SIZE);
Ewma avgTT205(MOVING_AVG_SIZE);
Ewma avgTT313(MOVING_AVG_SIZE);
Ewma avgTT444(MOVING_AVG_SIZE);
Ewma avgTT447(MOVING_AVG_SIZE);
Ewma avgTT1(MOVING_AVG_SIZE);
Ewma avgTT2(MOVING_AVG_SIZE);
Ewma avgTT3(MOVING_AVG_SIZE);

Ewma avgPTevoDischarge(MOVING_AVG_SIZE);
Ewma avgPTPSA1(MOVING_AVG_SIZE);
Ewma avgPTPSA2(MOVING_AVG_SIZE);
Ewma avgPTFinalDischarge(MOVING_AVG_SIZE);
Ewma avgACT1Pos(MOVING_AVG_SIZE);
Ewma avgACT2Pos(MOVING_AVG_SIZE);
Ewma avgPSAflow(MOVING_AVG_SIZE);
Ewma avgPT454(MOVING_AVG_SIZE);

struct Analog {
  String name;
  String key;
  Ewma avg;
  double* value;
  double prev;
  double raw;
  ADS7828 adc;
  int channel;
  int rating;
} AIdata[] = {
  {"COOLANT_TEMP_(GAS)","TT105 ",avgTT105,&TT105,0,0,adc1,1,-1},
  {"COOLANT_TEMP_(OIL)","TT205 ",avgTT205,&TT205,0,0,adc1,2,-1},
  {"GAS_SUCTION_TEMP","TT313 ",avgTT313,&TT313,0,0,adc1,3,-1},
  {"OIL_HTR_TEMP","TT444 ",avgTT444,&TT444,0,0,adc1,4,-1},
  {"OIL_SUMP_TEMP","TT447 ",avgTT447,&TT447,0,0,adc1,5,-1},
  {"RPSA1","TT1 ",avgTT1,&TT1,0,0,adc1,6,-1},
  {"RPSA2","TT2 ",avgTT2,&TT2,0,0,adc1,7,-1},
  {"SPARE","TT3 ",avgTT3,&TT3,0,0,adc1,8,-1},

  {"EVO_GAS_DISCHARGE_PRESSURE","PTevoDischarge",avgPTevoDischarge,&PTevoDischarge,0,0,adc1,0,10000},
  {"INTERMEDIATE_PSA_PRESS1","PTPSA1",avgPTPSA1,&PTPSA1,0,0,adc1,1,10000},
  {"INTERMEDIATE_PSA_PRESS2","PTPSA2",avgPTPSA2,&PTPSA2,0,0,adc1,2,10000},
  {"FINAL_GAS_DISCHARGE_PRESSURE","PTFinalDischarge",avgPTFinalDischarge,&PTFinalDischarge,0,0,adc1,3,10000},
  {"ACT1_POSITION","ACT1Pos",avgACT1Pos,&ACT1Pos,0,0,adc1,4,10000},
  {"ACT2_POSITION","ACT2Pos",avgACT2Pos,&ACT2Pos,0,0,adc1,5,10000},
  {"PSA1_FLOW","PSAflow",avgPSAflow,&PSAflow,0,0,adc1,6,10000},
  {"GAS_SUCTION_PRESSURE","PT454",avgPT454,&PT454,0,0,adc1,7,10000}
};
struct AnalogOut {
  String name;
  String key;
  double* value;
  double prev;
  int channel;
} AOdata[] = {
  {"RPSA1_ACT","ACT1",&ACT1,0,1},
  {"RPSA2_ACT","ACT2",&ACT2,0,2}
  //{"EVO_VFD_HZ","EVOHZ",&EVOHZ,0,,3,},
  //{"SPARE","SPARE",&SPARE,0,-1,4,-1}
}; 
int AOsize = 2;
int AIsize = 16;


struct RS485{
  String name;
  String key;
  double* value;
  double prev;
  int ID;
  int reg;
} RS485data[] = {
  {"PSA1_VFD_HZ","VFD1HZ",&VFD1HZ,0,PSA1_SLAVE_ID,speedAddr},
  //{"PSA1_VFD_HZ_FEEDBACK","VFD1HZfb",&VFD1HZfb,0,PSA1_SLAVE_ID,speedFbAddr},
  {"PSA1_VFD_ENABLE","VFD1EN",&VFD1EN,0,PSA1_SLAVE_ID,runAddr},
  {"PSA2_VFD_HZ","VFD2HZ",&VFD2HZ,0,PSA2_SLAVE_ID,speedAddr},
  //{"PSA1_VFD_HZ_FEEDBACK","VFD2HZfb",&VFD2HZfb,0,PSA2_SLAVE_ID,speedFbAddr},
  {"PSA2_VFD_ENABLE","VFD2EN",&VFD2EN,0,PSA2_SLAVE_ID,runAddr},
  {"EVO_VFD_HZ","EVOHZ",&EVOHZ,0,EVO_SLAVE_ID,speedAddr},
  //{"EVO_VFD_HZ_FEEDBACK","EVOHZfb",&EVOHZfb,0,EVO_SLAVE_ID,speedFbAddr},
  {"EVO_VFD_ENABLE","EVOEN",&EVOEN,0,EVO_SLAVE_ID,runAddr}
};
int RS485size = 6;

struct Digital {
  String name;
  String key;
  uint8_t* value;
  uint8_t prev;
  PI4IOE5V6534Q gpio;
  int addr;
} DOdata[] = {
  {"LOCALINT_LOOP","LSR",&LSR,0,gpio,P0_0},
  {"REMOTE_RESET","LSRRST",&LSRRST,0,gpio,P0_1},
  {"GREEN_PILOT","GPL",&GPL,0,gpio,P0_2},
  {"AMBER_PILOT","APL",&APL,0,gpio,P0_3},
  {"RED_PILOT","RPL",&RPL,0,gpio,P0_4},
  {"OIL_HEATER","HTR443",&HTR443,0,gpio,P0_5},
  {"COOLANT_FANS","CLTFAN",&CLTFAN,0,gpio,P0_6},
  {"COOLANT_PUMPS","CLTPMP",&CLTPMP,0,gpio,P0_7},
  {"DIVERTER_XV_TO_HX101B_/_OIL_CL_FAN","XV109",&XV109,0,gpio,P1_0},
  {"VE311_DROPOUT_XV","XV323",&XV323,0,gpio,P1_1},
  {"VE511_DROPOUT_XV","XV523",&XV523,0,gpio,P1_2},
  {"XV_RPSA_1_GAS_FEED","XVFD1",&XVFD1,0,gpio,P1_3},
  {"XV_RPSA_2_GAS_FEED","XVFD2",&XVFD2,0,gpio,P1_4},
  {"XV_RPSA_1_VENT_GAS","XVVNT1",&XVVNT1,0,gpio,P1_5},
  {"XV_RPSA_2_VENT_GAS","XVVNT2",&XVVNT2,0,gpio,P1_6},
  {"READYOP_R050_1","READYOP1",&READYOP1,0,gpio,P1_7},
  {"READYOP_R050_2","READYOP2",&READYOP2,0,gpio,P2_0}
  // {"RPSA1_VFD_ENABLE","VFD1EN",&VFD1EN,0,gpio,P2_1},
  // {"RPSA2_VFD_ENABLE","VFD2EN",&VFD2EN,0,gpio,P2_2},
  // {"EVO_VFD_ENABLE","EVOVFDEN",&EVOVFDEN,0,gpio,P2_3}
}, DIdata[] = {
  {"GLOB_ITLCK","GSR",&GSR,0,gpio,P2_4},
  {"LOCAL_ESTOP","ESTOP",&ESTOP,0,gpio,P2_5},
  {"GREEN_BUTTON","GBN",&GBN,0,gpio,P2_6},
  {"RED_BUTTON","RBN",&RBN,0,gpio,P2_7},
  {"LSR","LSROUT",&LSROUT,0,gpio,P3_0},
  {"COOLANT_FLOW_SWITCHES_1","CLTFLWSW",&CLTFLWSW,0,gpio,P3_1},
  {"HIGH_LEVEL_DROPOUT_SW","LSH321",&LSH321,0,gpio,P3_2},
  {"LOW_LEVEL_DROPOUT_SW","LSL322",&LSL322,0,gpio,P3_3},
  {"HIGH_LEVEL_DROPOUT_SW","LSH521",&LSH521,0,gpio,P3_4},
  {"LOW_LEVEL_DROPOUT_SW","LSL522",&LSL522,0,gpio,P3_5},
  {"READYIP_R050_1","READYIP1",&READYIP1,0,gpio,P3_6},
  {"READYIP_R050_2","READYIP2",&READYIP2,0,gpio,P3_7}
};
int DOsize = 17;
int DIsize = 12;
uint8_t mcpExist = 0;
double mxy = 0;




