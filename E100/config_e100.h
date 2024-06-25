#if !defined(_CONFIG_E100_H)
  #define  _CONFIG_E100_H

  #include <stdint.h>

  typedef union
  {
    float number;
    uint16_t words[2];
    uint8_t bytes[4];
    uint32_t uint_value;
  } FLOATUNION_t;

  #define MSB         16
  #define LSB         0
  #define AS_INT      32

  // PM-904 (The pressure reading from FM-904)
  #define   CONFIG_PARAM_RELAYS                     1   // 1 & 2 - 32 bits
  #define   RELAY_BIT_GREEN_PILOT                   0   // Green pilot light
  #define   RELAY_BIT_AMBER_PILOT                   1   // Amber pilot light
  #define   RELAY_BIT_RED_PILOT                     2   // Red pilot light
  #define   RELAY_BIT_OIL_HEATER                    3   // Oil heater P0_5, R24.6
  #define   RELAY_BIT_COOLANT_FANS                  4   // Coolant fans - R24.7 DO.7/P0_7
  #define   RELAY_BIT_COOLANT_PUMP                  5   // Coolant pumps - R24.8 DO.8/P0_8
  #define   RELAY_BIT_REMOTE_RESET                  6   // Remote reset - R24.2/P0_1
  #define   RELAY_BIT_LOCAL_INTERLOCK               7   // Local interlock - R24.1/P0_0
  #define   RELAY_BIT_DIVERTER_XV                   8   // DIVERTER XV TO HX101B / OIL CL FAN R24.9 P1_0
  #define   RELAY_BIT_VE311                         9   // VE311 Drop-out XV R24.10
  #define   RELAY_BIT_VE511                         10  // VE511 Drop-out XV R24.11
  #define   RELAY_BIT_XV_RPSA1_GAS_FEED             11  // XV RPSA.1 Gas feed R24.12
  #define   RELAY_BIT_XV_RPSA2_GAS_FEED             12  // XV RPSA.2 Gas feed R24.13
  #define   RELAY_BIT_XV_RPSA1_VENT_GAS             13  // XV RPSA.1 Vent gas R24.14
  #define   RELAY_BIT_XV_RPSA2_VENT_GAS             14  // XV RPSA.2 Vent gas R24.15
  #define   RELAY_BIT_READY_R050_1                  15  // Ready to R050 - R24.16
  #define   RELAY_BIT_READY_R050_2                  16  // Ready to R050 - R24.17
  #define   RELAY_BIT_PSA1_VFD_RUN                  17  // PSA VFD Run 1 - R24.18
  #define   RELAY_BIT_PSA2_VFD_RUN                  18  // PSA VFD Run 2 - R24.19
  #define   RELAY_BIT_EVO_VFD_RUN                   19  // EVO VFD Run - R24.20

  #define   CONFIG_PARAM_INPUTS                     3   // Digital input states
  #define   INPUT_BIT_GLOBAL_INT                    0   // Global interlock - R24.0 NO1 - DI.1
  #define   INPUT_BIT_LOCAL_ESTOP                   1   // Local E-STOP TB5.2
  #define   INPUT_BIT_GREEN_BUTTON                  2   // Green button switch - TB6.4
  #define   INPUT_BIT_RED_BUTTON                    3   // Red button switch - TB8.4
  #define   INPUT_BIT_LSR_IP                        4   // LSR IP TB12.10
  #define   INPUT_BIT_COOLANT_FS                    5   // Coolant flow switches TB13.4
  #define   INPUT_BIT_HIGH_LEVEL_DO_1               6   // High level drop out R24.22
  #define   INPUT_BIT_LOW_LEVEL_DO_1                7   // Low level drop out R24.23
  #define   INPUT_BIT_HIGH_LEVEL_DO_2               8   // High level drop out R24.24
  #define   INPUT_BIT_LOW_LEVEL_DO_2                9   // Low level drop out R24.25
  #define   INPUT_BIT_READY_IP_R050_1               10  // Ready IP R050 R24.26 #1
  #define   INPUT_BIT_READY_IP_R050_2               11  // Ready IP R050 R24.27 #2

  // please check the numbers 
  // EVO GAS PRESSURE 
  #define   CONFIG_PARAM_EVO_GAS_PX                 9       //EVO gas discharge TB44
  #define   CONFIG_PARAM_EVO_GAS_STOP_MIN           841     // Future use only!
  #define   CONFIG_PARAM_EVO_GAS_IDLE_MIN           842     // Future use only!
  #define   CONFIG_PARAM_EVO_GAS_RUN_MIN            843     // Future use only!
  #define   CONFIG_PARAM_EVO_GAS_RUN_MAX            844     // Future use only!
  #define   CONFIG_PARAM_EVO_GAS_IDLE_MAX           845     // Future use only!
  #define   CONFIG_PARAM_EVO_GAS_STOP_MAX           846     // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_STOP_MIN         60      // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_IDLE_MIN         65      // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_RUN_MIN          70      // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_RUN_MAX          110     // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_IDLE_MAX         120     // Future use only!
  #define   SETPOINT_EVO_GAS_ENTER_STOP_MAX         150     // Future use only!

  // PSA PRESS 1 PRESSURE
  #define   CONFIG_PARAM_PSA_PRESS_1_PX                 10  //Intermediate PSA press 1 TB45
  #define   CONFIG_PARAM_PSA_PRESS_1_STOP_MIN           851     // Future use only!
  #define   CONFIG_PARAM_PSA_PRESS_1_IDLE_MIN           852     // Future use only!
  #define   CONFIG_PARAM_PSA_PRESS_1_RUN_MIN            853     // Future use only!
  #define   CONFIG_PARAM_PSA_PRESS_1_RUN_MAX            854     // Future use only!
  #define   CONFIG_PARAM_PSA_PRESS_1_IDLE_MAX           855     // Future use only!
  #define   CONFIG_PARAM_PSA_PRESS_1_STOP_MAX           856     // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_STOP_MIN         -1      // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_IDLE_MIN         -1      // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_RUN_MIN          -1      // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_RUN_MAX          2000     // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_IDLE_MAX         2000     // Future use only!
  #define   SETPOINT_PSA_PRESS_1_ENTER_STOP_MAX         2000     // Future use only!

  //PSA PRESS 2 PRESSURE 
  #define   CONFIG_PARAM_PSA_PRESS_2_PX               11  //Intermediate PSA press 2 TB46
  #define   CONFIG_PARAM_PSA_PRESS_2_STOP_MIN         881
  #define   CONFIG_PARAM_PSA_PRESS_2_IDLE_MIN         882
  #define   CONFIG_PARAM_PSA_PRESS_2_RUN_MIN          883
  #define   CONFIG_PARAM_PSA_PRESS_2_RUN_MAX          884
  #define   CONFIG_PARAM_PSA_PRESS_2_IDLE_MAX         885
  #define   CONFIG_PARAM_PSA_PRESS_2_STOP_MAX         886   
  #define   SETPOINT_PSA_PRESS_2_ENTER_STOP_MIN      -100    // N/A
  #define   SETPOINT_PSA_PRESS_2_ENTER_IDLE_MIN      -20     // S1 IDLE
  #define   SETPOINT_PSA_PRESS_2_ENTER_RUN_MIN       -10     // N/A
  #define   SETPOINT_PSA_PRESS_2_ENTER_RUN_MAX        500
  #define   SETPOINT_PSA_PRESS_2_ENTER_IDLE_MAX       600     // S1 IDLE
  #define   SETPOINT_PSA_PRESS_2_ENTER_STOP_MAX       700

  #define   CONFIG_PARAM_FINAL_GAS_DISCHARGE_PX     12  //Final gas discharge pressure  TB47
  #define   CONFIG_PARAM_ACT_1_PX                   69  //ACT 1 Postion TB48
  #define   CONFIG_PARAM_ACT_2_PX                   71  //ACT 2 Position TB49
  #define   CONFIG_PARAM_PSA_1_PX                   73  //PSA 1 FLOW  TB50
  #define   CONFIG_PARAM_GAS_SUCTION_PX             18  //GAS Suction pressure TB51

  #define   CONFIG_PARAM_TC105                      29  // Coolant temperature (Gas) TB32
  #define   CONFIG_PARAM_TC205                      30  // Coolant tempature (Oil) TB33
  #define   CONFIG_PARAM_TC313                      31  // Gas suction temp TB34
  #define   CONFIG_PARAM_TC444                      32  // Gas suction temp TB34
  #define   CONFIG_PARAM_TC447                      33  // Oil sump temp - TB36
  #define   CONFIG_PARAM_TC_RPSA1                   34  // TB37
  #define   CONFIG_PARAM_TC_RPSA2                   35  // TB38
  #define   CONFIG_PARAM_TC_SPARE                   36  // TB39

  
  // Analog Outputs Parameters
  #define   CONFIG_PARAM_AO_1                       75  //TB40
  #define   CONFIG_PARAM_AO_2                       77  //TB41
  #define   CONFIG_PARAM_AO_3                       79  //TB42
  #define   CONFIG_PARAM_AO_4                       81  //TB43
  #define   CONFIG_PARAM_VFD_ON_OFF_SETTING         83  // TB54
  #define   CONFIG_PARAM_VFD_SPEED_SETTING          85 // TB54

  #define   CONFIG_PARAM_OP_STATE_ALL               91   // Current operational states (Soft variable)
  #define   OP_STATE_MANUAL_CONTROL                 0    // 0 = not manual control (Auto), 1 = Manual control (Auto off)
  #define   OP_STATE_GREEN_BUTTON                   7    // Remote green button press
  #define   OP_STATE_AMBER_BUTTON                   9    // Remote amber button press
  #define   OP_STATE_RED_BUTTON                     11   // Remote red button press

  #define   CONFIG_PARAM_PRODUCTION_EVO_DELTA_STEP  2003
  #define   CONFIG_PARAM_PRODUCTION_EVO_DELTA_TIME  2005

  void init_config(void);
  float get_config_parameter(int);
  bool test_config_parameter(int, int);
  uint16_t get_config_parameter(int, int);
  void set_config_parameter(int, float);
  void set_config_parameter(int, uint16_t, int);
  void set_config_bit(int, uint16_t, int);
#endif
