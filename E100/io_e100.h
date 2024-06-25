/*
   Author: Christopher Glanville
   io_c200.h configuration header file. To acompany io_c200.cpp
   Date: 25/5/24
   updated : 25/5/24
*/

#if !defined(_IO_E100_H)
  #define  _IO_E100_H
  #include "e100.h"

  #define BUTTON_PRESS          true
  #define BUTTON_RELEASE        false

  void io_setup(void);
  void digital_io_loop(void);
  void analog_io_loop(void);
  // bool get_estop_button_state(void);
  // void set_green_pilot_state(bool);
  // void set_amber_pilot_state(bool);
  // void set_amber_pilot_state_xx(bool);
  // void set_red_pilot_state(bool);
  // void set_remote_reset(bool);
  // bool getHydraulicFluidLvl(void);
  
  // bool setHydraulicPump(bool);
  // bool setCoolingSystem(bool);
  // bool setCoolingFans(int, bool);

  // void setSuctionSolenoid(bool);

  // void toggleStroking(bool, bool, bool);
  // void toggleStrokingA(bool, bool);
  // void toggleStrokingB(bool, bool);
  // void toggleStroking2(bool, bool, bool);
#endif
