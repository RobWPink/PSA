#if !defined(_MODBUS_E100_H)
  #define  _MODBUS_E100_H

  #define     TCP_BUFFER                    1 
  #define     RS_485_BUFFER                 2
  #define     BLE_BUFFER                    3
  #define     SLAVE_ID                      1

  #define MODBUS_REG_BUTTONS_RGA                    0
  #define MODBUS_REG_RELAYS                         1
  #define MODBUS_REG_RELAYS_2                       2
  #define MODBUS_REG_INPUTS                         3 // Digital inputs
  #define MODBUS_REG_EVO_GAS_PX                     9
  #define MODBUS_REG_PSA_PRESS_1_PX                 10  //Intermediate PSA press 1 TB45
  #define MODBUS_REG_PSA_PRESS_2_PX                 11  //Intermediate PSA press 2 TB46
  #define MODBUS_REG_FINAL_GAS_DISCHARGE_PX         12  //Final gas discharge pressure  TB47
  #define MODBUS_REG_GAS_SUCTION_PX                 13  //Gas suction pressure TB51
  #define MODBUS_REG_ACT_1_PX                       69  //ACT 1 Postion TB48
  #define MODBUS_REG_ACT_2_PX                       71  //ACT 2 Position TB49
  #define MODBUS_REG_PSA_1_PX                       73  //PSA 1 FLOW  TB50
  #define MODBUS_REG_TC105                          29
  #define MODBUS_REG_TC205                          30  // Coolant tempature (Oil) TB33
  #define MODBUS_REG_TC313                          31  // Gas suction temp TB34
  #define MODBUS_REG_TC444                          32  // Gas suction temp TB34
  #define MODBUS_REG_TC447                          33  // Oil sump temp - TB36
  #define MODBUS_REG_TC_RPSA1                       34  // TB37
  #define MODBUS_REG_TC_RPSA2                       35  // TB38
  #define MODBUS_REG_TC_SPARE                       36  // TB39
  #define MODBUS_REG_AO001                          75  // TB41
  #define MODBUS_REG_AO002                          77  // TB42
  #define MODBUS_REG_AO003                          79  // TB43
  #define MODBUS_REG_AO004                          81  // TB44
  #define MODBUS_REG_VFD_ON_OFF_SETTING             83  // TB54
  #define MODBUS_REG_VFD_SPEED_SETTING              85  // TB54
  #define MODBUS_REG_OP_STATE_ALL                   91
  #define MODBUS_REG_SLAVE_ID                       2000
  #define MODBUS_REG_SERIAL_NUMBER                  2001

  #define MODBUS_REG_PRODUCTION_EVO_DELTA_STEP      2003
  #define MODBUS_REG_PRODUCTION_EVO_DELTA_TIME      2005

  #if defined(HARDWARE_GIGA)
    void modbuxRxData(uint8_t, int);
    int modbus_loop(int);
    int get_tx_bytes(int);
    uint8_t * get_tx_buffer(int);
  #endif
#endif