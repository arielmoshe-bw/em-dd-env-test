//
// Created by arielmoshe-bw on 12/10/23.
//

#ifndef DD_ENV_TEST_MANAGER_H
#define DD_ENV_TEST_MANAGER_H

#include <Arduino.h>
#include <ArduinoLog.h>

#include "../DirectDriveSteeringWheel/direct_drive_steering_wheel.hpp"
#include "../SingletonTimer/singleton_timer.hpp"

// Enum defining different states of the Direct Drive Environment Test
enum DirectDriveEnvironmentTestState : uint8_t
{
  POWER_UP_STATE,
  ROW_DRIVING_STATE,
  TURN_DRIVING_STATE,
};

class DirectDriveEnvironmentTestManager
{
private:
  DirectDriveSteeringWheel* direct_drive_steering_wheel_ = nullptr;

  SingletonTimer& timer_;
  
  DirectDriveEnvironmentTestState current_state_;
  
  unsigned long main_system_timer_in_ms_ = 0;
  unsigned long main_loop_timestamp_in_ms_ = 0;
  const unsigned long MAIN_LOOP_SPIN_RATE_IN_MS_ = 1000 / 250;
  unsigned long turn_loop_timestamp_in_ms_ = 0;
  const unsigned long TURN_SPIN_RATE_IN_MS_ = 3000;
  
public:
  DirectDriveEnvironmentTestManager();
  
  static DirectDriveEnvironmentTestManager &getInstance();
  
  // Delete copy constructor and assignment operator to enforce singleton
  DirectDriveEnvironmentTestManager(const DirectDriveEnvironmentTestManager &) = delete;
  void operator=(const DirectDriveEnvironmentTestManager &) = delete;
  
private:
  void handleState();
  void handlePowerUpState();
  void handleRowDrivingState();
  void handleTurnDrivingState();
  
  bool isLoopTickPossible(volatile unsigned long &last_loop_timestamp_in_ms, const unsigned long loop_spin_rate_in_ms);
  
public:
  void setup();
  void run();
  
  void setState(const DirectDriveEnvironmentTestState newState);
  DirectDriveEnvironmentTestState getState() const {return current_state_;};
};

#endif
