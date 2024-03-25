//
// Created by arielmoshe-bw on 12/10/23.
//

#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include "src/DirectDriveEnvironmentTestManager/dd_env_test_manager.h"

volatile bool is_received_new_data_ = false;
String received_data_buffer_ = "";
String received_data_ = "";

DirectDriveEnvironmentTestManager &dd_env_test_manager = DirectDriveEnvironmentTestManager::getInstance();

void setup()
{
  dd_env_test_manager.setup();
}

void loop()
{
  dd_env_test_manager.run();
}
