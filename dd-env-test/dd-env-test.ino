//
// Created by arielmoshe-bw on 12/10/23.
//

#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include "src/DirectDriveEnvironmentTestManager/dd_env_test_manager.h"

DirectDriveEnvironmentTestManager &dd_env_test_manager = DirectDriveEnvironmentTestManager::getInstance();

void setup()
{
  dd_env_test_manager.setup();
}

void loop()
{
  dd_env_test_manager.run();
}
