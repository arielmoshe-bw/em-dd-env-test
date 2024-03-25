//
// Created by arielmoshe-bw on 12/10/23.
//

#include "dd_env_test_manager.h"

DirectDriveEnvironmentTestManager::DirectDriveEnvironmentTestManager()
  :
  current_state_(DirectDriveEnvironmentTestState::POWER_UP_STATE),
  timer_(SingletonTimer::getInstance())
{

}

DirectDriveEnvironmentTestManager &DirectDriveEnvironmentTestManager::getInstance()
{
  static DirectDriveEnvironmentTestManager instance;
  return instance;
}

void DirectDriveEnvironmentTestManager::setup()
{
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  
  direct_drive_steering_wheel_ = new DirectDriveSteeringWheel();
}

void DirectDriveEnvironmentTestManager::run()
{
  main_system_timer_in_ms_ = timer_.getCurrentTime();
  
  checkReceivedData();
  if (isLoopTickPossible(main_loop_timestamp_in_ms_, MAIN_LOOP_SPIN_RATE_IN_MS_))
  {
    handleState();
  }
}

void DirectDriveEnvironmentTestManager::setState(const DirectDriveEnvironmentTestState new_state)
{
  current_state_ = new_state;
}

void DirectDriveEnvironmentTestManager::handleState()
{
  if(not isHostStop())
  {
    direct_drive_steering_wheel_->tick();
    
    if (direct_drive_steering_wheel_->getOperationMode() == DirectDriveSteeringWheel::MotorOperationMode::STABLE &&
      current_state_ != POWER_UP_STATE)
    {
      direct_drive_steering_wheel_->performContinuousBuiltInTest();
      printData();
    }
    
    switch (current_state_)
    {
      case POWER_UP_STATE:
      {
        handlePowerUpState();
        break;
      }
      
      case WAIT_FOR_HOST_STATE:
      {
        handleWaitForHostState();
        break;
      }
      
      case ROW_DRIVING_STATE:
      {
        handleRowDrivingState();
        break;
      }
      
      case TURN_DRIVING_STATE:
      {
        handleTurnDrivingState();
        break;
      }
      
      default:
      {
        break;
      }
    }
  }
}

void DirectDriveEnvironmentTestManager::handlePowerUpState()
{
  direct_drive_steering_wheel_->performPowerUpBuiltInTest();
  direct_drive_steering_wheel_->releaseActuator();
  
  setState(WAIT_FOR_HOST_STATE);
}

void DirectDriveEnvironmentTestManager::handleWaitForHostState()
{
  if(isHostReady())
  {
    setState(ROW_DRIVING_STATE);
  }
}

void DirectDriveEnvironmentTestManager::handleRowDrivingState()
{
  static bool is_left = false;
  
  if (direct_drive_steering_wheel_->getOperationMode() == DirectDriveSteeringWheel::MotorOperationMode::STABLE)
  {
    if (isLoopTickPossible(row_loop_timestamp_in_ms_, ROW_SPIN_RATE_IN_MS_))
    {
      if (row_counter_ > ROW_NUM_ITERATIONS_)
      {
        row_counter_ = 0;
        setState(TURN_DRIVING_STATE);
      }
      else
      {
        //Log.infoln("handleRowDrivingState row_counter_ = %d", row_counter_);
        if (is_left)
        {
          direct_drive_steering_wheel_->setRpmPercentage(RPM_PERCENTAGE_);
        }
        else
        {
          direct_drive_steering_wheel_->setRpmPercentage(-RPM_PERCENTAGE_);
        }
        is_left ^= true;
      }
      row_counter_++;
    }
  }
}

void DirectDriveEnvironmentTestManager::handleTurnDrivingState()
{
  static bool is_left = false;
  
  if (direct_drive_steering_wheel_->getOperationMode() == DirectDriveSteeringWheel::MotorOperationMode::STABLE)
  {
    if (isLoopTickPossible(turn_loop_timestamp_in_ms_, TURN_SPIN_RATE_IN_MS_))
    {
      //Log.infoln("handleTurnDrivingState turn_counter_= %d", turn_counter_);
      if (turn_counter_ > TURN_NUM_ITERATIONS_)
      {
        turn_counter_ = 0;
        setState(ROW_DRIVING_STATE);
      }
      else
      {
        if (is_left)
        {
          direct_drive_steering_wheel_->setRpmPercentage(RPM_PERCENTAGE_);
        }
        else
        {
          direct_drive_steering_wheel_->setRpmPercentage(-RPM_PERCENTAGE_);
        }
        is_left ^= true;
      }
      turn_counter_++;
    }
  }
}

bool DirectDriveEnvironmentTestManager::isHostReady()
{
  if (dd_is_received_new_data_)
  {
    if (received_data_.equals("ready\n"))
    {
      dd_is_received_new_data_ = false;
      Serial.println("start sampling");
      return true;
    }
  }
  return false;
}

bool DirectDriveEnvironmentTestManager::isHostStop()
{
  return received_data_.equals("stop\n");
}

void DirectDriveEnvironmentTestManager::checkReceivedData()
{
  if (is_received_new_data_)
  {
    received_data_ = received_data_buffer_;
    dd_is_received_new_data_ = true;
    is_received_new_data_ = false;
    received_data_buffer_ = "";
  }
}

void DirectDriveEnvironmentTestManager::printData()
{
  if(direct_drive_steering_wheel_->getKaFlag())
  {
    direct_drive_steering_wheel_->setKaFlag(true);
    
    Serial.print("Current in amps:");
    Serial.println(direct_drive_steering_wheel_->getMotorCurrentInAmps());
    Serial.print("Voltage in volts:");
    Serial.println(direct_drive_steering_wheel_->getMotorVoltageInVolts());
    Serial.print("Temperature in degree:");
    Serial.println(direct_drive_steering_wheel_->getMotorTemperatureInDegree());
    Serial.print("Encoder velocity in rpm:");
    Serial.println(direct_drive_steering_wheel_->getMotorEncoderVelocity());
    Serial.print("Encoder position in degree:");
    Serial.println(direct_drive_steering_wheel_->getMotorEncoderPosition());
    Serial.print("Fault code:");
    Serial.println(direct_drive_steering_wheel_->getMotorFaultCode(), HEX);
    Serial.println();
  }
}

bool DirectDriveEnvironmentTestManager::isLoopTickPossible(volatile unsigned long &last_loop_timestamp_in_ms,
                                                           const unsigned long loop_spin_rate_in_ms)
{
  if (abs(main_system_timer_in_ms_ - last_loop_timestamp_in_ms) > loop_spin_rate_in_ms)
  {
    last_loop_timestamp_in_ms = main_system_timer_in_ms_;
    return true;
  }
  return false;
}

void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    
    // add it to the inputString:
    received_data_buffer_ += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
    {
      is_received_new_data_ = true;
    }
  }
}