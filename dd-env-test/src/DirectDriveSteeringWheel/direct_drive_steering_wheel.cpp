//
// Created by daniel-bw on 7/26/23.
//

#include "direct_drive_steering_wheel.hpp"

DirectDriveSteeringWheel::DirectDriveSteeringWheel()
  :
  mcp2515(53),
  timer_(SingletonTimer::getInstance())
{
  Log.infoln("Direct drive Steering wheel is initializing");
  initializeCanCommunication();
  lockActuator();
}

DirectDriveSteeringWheel::~DirectDriveSteeringWheel()
{
}

void DirectDriveSteeringWheel::initializeCanCommunication()
{
  Log.infoln("initializeCanCommunication");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

bool DirectDriveSteeringWheel::performPowerUpBuiltInTest()
{
  Log.infoln(F("Direct Drive steering wheel motor Power Up BIT starting"));
  
  initializePowerUpSequence();

  motor_keep_alive_is_healthy_ = false;

  resetFaultConsecutiveCounter(MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR);
  resetFaultConsecutiveCounter(MalfunctionsBitId::DD_STEERING_UNRESPONSIVE_ERROR);
}

bool DirectDriveSteeringWheel::performContinuousBuiltInTest()
{
  if (not isVersionMotorUpdated())
  {
    sendMotorQuery(program_version_payload_data_);
  }

  checkForMessageFromMotor();

  handleErrorsRaisedByMotor();
  motorKeepAliveContinuousBuiltInTest();
  overCurrentContinuousBitProcess();

  publishKeepAliveToMotorWhenDisabled();
  sendMotorQuery(motor_voltage_query_payload_data_);
  sendMotorQuery(motor_temperature_query_payload_data_);
}

void DirectDriveSteeringWheel::lockActuator()
{
  command(0);
  sendMotorQuery(disable_motor_payload_data_);
  is_motor_enabled_ = false;
}

void DirectDriveSteeringWheel::releaseActuator()
{
  if (not is_motor_enabled_)
  {
    sendMotorQuery(enable_motor_payload_data_);
    is_motor_enabled_ = true;
    command(0);
  }
}

bool DirectDriveSteeringWheel::command(int rpm_percentage)
{
  if (not is_motor_enabled_)
  {
    return false;
  }
  
  int pwm_2s_complement = abs(rpm_percentage);
  
  if (rpm_percentage < 0)
  {
    pwm_2s_complement = ~(abs(rpm_percentage)) + 1;
  }
  
  uint32_t mixedEndianHex =
    ((uint32_t(pwm_2s_complement) & 0xFF00) << 8) | ((uint32_t(pwm_2s_complement) & 0xFF000000) >> 24)
      | ((uint32_t(pwm_2s_complement) & 0xFF) << 24) | ((uint32_t(pwm_2s_complement) & 0xFF0000) >> 8);
  
  speed_command_motor_payload_data_[4] = (mixedEndianHex >> 16) & 0xFF;
  speed_command_motor_payload_data_[5] = (mixedEndianHex >> 24) & 0xFF;
  speed_command_motor_payload_data_[6] = mixedEndianHex & 0xFF;
  speed_command_motor_payload_data_[7] = (mixedEndianHex >> 8) & 0xFF;
  
  sendMotorQuery(speed_command_motor_payload_data_);

  return true;
}

bool DirectDriveSteeringWheel::changeMotorOperationMode(MotorOperationMode new_motor_operation_mode)
{
  if (new_motor_operation_mode == motor_operation_mode_)
  {
    return false;
  }

  Log.infoln(F("Motor operation mode changed from %s to %s"),
             operationModeToString(motor_operation_mode_),
             operationModeToString(new_motor_operation_mode));

  motor_operation_mode_ = new_motor_operation_mode;

  return true;
}

void DirectDriveSteeringWheel::tick()
{
  main_system_timer_in_ms_ = timer_.getCurrentTime();

  checkForMessageFromMotor();
  
  switch (motor_operation_mode_)
  {
    case MotorOperationMode::POWER_UP_SEQUENCE:
    {
      powerUpSequenceProcess();
      break;
    }
    
    case MotorOperationMode::STABLE:
    {
      stableOperationModeProcess();
      break;
    }
    
    case MotorOperationMode::FAULT:
    {
      break;
    }
    
    default:
    {
      break;
    }
  }
}

void DirectDriveSteeringWheel::setRpmPercentage(int rpm_percentage)
{
  if(abs(rpm_percentage) > 400)
  {
    rpm_percentage = rpm_percentage > 0 ? 400 : - 400;
  }
  rpm_percentage_ = rpm_percentage;
}

bool DirectDriveSteeringWheel::setStableOperationMode()
{
  changeMotorOperationMode(STABLE);
}

void DirectDriveSteeringWheel::stableOperationModeProcess()
{
  command(rpm_percentage_);

  checkForMessageFromMotor();
}

void DirectDriveSteeringWheel::parseResponseFromMotor(const can_frame &response_can_frame)
{
  uint32_t response_data_type =
    ((uint32_t) response_can_frame.data[0]) << 24 | ((uint32_t) response_can_frame.data[1]) << 16 | ((uint32_t) response_can_frame.data[2]) << 8 | ((uint32_t) response_can_frame.data[3]);

  switch (response_data_type)
  {
    case MOTOR_PROGRAM_VERSION_PREFIX_:
    {
      motor_program_version_[0] = response_can_frame.data[4];
      motor_program_version_[1] = response_can_frame.data[5];
      motor_program_version_[2] = response_can_frame.data[6];
      motor_program_version_[3] = response_can_frame.data[7];

      Log.infoln("%d.%d.%d.%d", motor_program_version_[0], motor_program_version_[1], motor_program_version_[2], motor_program_version_[3]);

      break;
    }
    case MOTOR_CURRENT_RESPONSE_PREFIX_:
    {
      motor_current_in_amps_ = response_can_frame.data[4];
      break;
    }
    case MOTOR_VOLTAGE_RESPONSE_PREFIX_:
    {
      motor_voltage_in_volts_ = response_can_frame.data[4];
      break;
    }
    case MOTOR_TEMPERATURE_RESPONSE_PREFIX_:
    {
      motor_temperature_in_deg_ = response_can_frame.data[4];
      break;
    }
    case MOTOR_ENCODER_VELOCITY_RESPONSE_PREFIX_:
    {
      motor_encoder_velocity_ = response_can_frame.data[5] << 8 | response_can_frame.data[4];
      break;
    }
    case MOTOR_ENCODER_POSITION_RESPONSE_PREFIX_:
    {
      motor_encoder_position_ =
        response_can_frame.data[7] << 24 | response_can_frame.data[6] << 16 | response_can_frame.data[5] << 8
          | response_can_frame.data[4];
      break;
    }
    case FAULT_RESPONSE_PREFIX_:
    {
      setMotorFaultCode(response_can_frame.data[6] << 8 | response_can_frame.data[7]);
      break;
    }
    case 0x60002000:// TODO: Get from the datasheet the name of the prefix here
    {
      break;
    }
    case MOTOR_ENABLED_CONFIRMATION_PREFIX_:
    {
      Log.infoln(F("DD Motor enabled confirmed"));
      clearMotorFaultCode(MOTOR_DISABLED_FAULT_CODE_);
      break;
    }
    case MOTOR_DISABLED_CONFIRMATION_PREFIX_:
    {
      if (not checkIfMotorDisabledFaultCodeFlagged())
      {
        Log.infoln(F("DD Motor disabled confirmed"));
        setMotorFaultCode(MOTOR_DISABLED_FAULT_CODE_);
      }
      break;
    }
    default:
      return;
  }
}

bool DirectDriveSteeringWheel::motorKeepAliveContinuousBuiltInTest()
{
  if (isPowerUpSequenceInProgress())
  {
    return false;
  }

  if (main_system_timer_in_ms_ - last_motor_ka_msg_timestamp >= MOTOR_KEEP_ALIVE_TIMEOUT_IN_MS_)
  {
    if (motor_keep_alive_is_healthy_)
    {
      Log.infoln(F("DD motor Keep Alive freeze for %dms"), MOTOR_KEEP_ALIVE_TIMEOUT_IN_MS_);
      motor_keep_alive_is_healthy_ = false;
      clearAllMotorFaultCodes();
    }
  }
  else
  {
    if (not motor_keep_alive_is_healthy_)
    {
      Log.infoln(F("DD motor Keep Alive restored !"));
      motor_keep_alive_is_healthy_ = true;
      if (is_motor_enabled_ and checkIfMotorDisabledFaultCodeFlagged())
      {
        sendMotorQuery(enable_motor_payload_data_);
      }
    }
  }

  return motor_keep_alive_is_healthy_;
}

void DirectDriveSteeringWheel::resetFaultConsecutiveCounter(MalfunctionsBitId malfunction_id)
{
  switch (malfunction_id) {
    case MalfunctionsBitId::DD_STEERING_UNRESPONSIVE_ERROR: {
      consecutive_unresponsive_faults_ = 0;
      Log.warningln(F("Unresponsive Direct drive fault consecutive counter has been reset !"));
      break;
    }
    case MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR: {
      consecutive_over_current_faults_ = 0;
      Log.warningln(F("Direct drive Over Current fault consecutive counter has been reset !"));
      break;
    }
    default: {
      Log.errorln(F("Unknown Fault to reset on Direct drive"));
      break;
    }
  }
}

void DirectDriveSteeringWheel::increaseConsecutiveCounterToFault(MalfunctionsBitId malfunction_id)
{
  switch (malfunction_id) {
    case MalfunctionsBitId::DD_STEERING_UNRESPONSIVE_ERROR: {
      consecutive_unresponsive_faults_++;
      Log.warningln(F("Unresponsive Direct drive fault consecutive counter increased to %d!"), consecutive_unresponsive_faults_);
      break;
    }
    case MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR: {
      consecutive_over_current_faults_++;
      Log.warningln(F("Direct drive Over Current fault consecutive counter increased to %d!"), consecutive_over_current_faults_);
      break;
    }
    default: {
      Log.errorln(F("Unknown Fault to increase to on Direct drive"));
      break;
    }
  }
}

void DirectDriveSteeringWheel::checkIfFaultReachedMaxConsecutive(MalfunctionsBitId malfunction_id)
{
  switch (malfunction_id) {
    case MalfunctionsBitId::DD_STEERING_UNRESPONSIVE_ERROR: {
      if (consecutive_unresponsive_faults_ >= MAX_CONSECUTIVE_UNRESPONSIVE_FAULTS)
      {
        Log.warningln(F("Unresponsive Direct drive has occurred %d times consecutively"), MAX_CONSECUTIVE_UNRESPONSIVE_FAULTS);
      }
      break;
    }
    case MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR: {
      if (consecutive_over_current_faults_ >= MAX_CONSECUTIVE_OVER_CURRENT_FAULTS)
      {
        
        Log.warningln(F("Direct drive Over Current has occurred %d times consecutively"), MAX_CONSECUTIVE_OVER_CURRENT_FAULTS);
      }
      break;
    }
    default: {
      Log.errorln(F("Unknown Fault to check on Direct drive"));
      break;
    }
  }
}

bool DirectDriveSteeringWheel::isVersionMotorUpdated()
{
  if (motor_program_version_[0] == 0 and motor_program_version_[1] == 0 and motor_program_version_[2] == 0 and motor_program_version_[3] == 0)
  {
    return false;
  }
  return true;
}
void DirectDriveSteeringWheel::parseKeepAliveData(const can_frame &response_can_frame)
{
  motor_encoder_position_ = response_can_frame.data[0] << 8 | response_can_frame.data[1];
  motor_encoder_velocity_ = response_can_frame.data[2] << 8 | response_can_frame.data[3];
  motor_current_in_amps_ = response_can_frame.data[4] << 8 | response_can_frame.data[5];
  setMotorFaultCode(response_can_frame.data[6] << 8 | response_can_frame.data[7]);

  if (motor_encoder_velocity_ > 0.0)
  {
    motor_has_moved_at_least_once_ = true;
  }

  motor_encoder_velocity_ = float(motor_encoder_position_ - last_motor_encoder_position_) / (0.001 * (main_system_timer_in_ms_ - last_ka_timestamp_));
  //Log.infoln(F("%l, %l, %F, %l"), motor_encoder_position_, last_motor_encoder_position_, motor_encoder_velocity_,
  //           main_system_timer_in_ms_ - last_ka_timestamp_);
  last_motor_encoder_position_ = motor_encoder_position_;
  last_ka_timestamp_ = main_system_timer_in_ms_;
}

void DirectDriveSteeringWheel::initializePowerUpSequence(bool enable_motor_once_completed)
{
  current_power_up_sequence_stage_ = INITIATE_POWER_UP;
  enable_motor_once_power_up_completed_ = enable_motor_once_completed;
  Log.warningln(F("Initializing motor Power up sequence"));
  changeMotorOperationMode(MotorOperationMode::POWER_UP_SEQUENCE);
}

bool DirectDriveSteeringWheel::isPowerUpSequenceInProgress()
{
  return current_power_up_sequence_stage_ != PowerUpSequenceStages::POWER_UP_COMPLETED
    and current_power_up_sequence_stage_ != PowerUpSequenceStages::PENDING_POWER_UP_INIT;
}

void DirectDriveSteeringWheel::powerUpSequenceProcess()
{
  switch (current_power_up_sequence_stage_) {
    case PowerUpSequenceStages::INITIATE_POWER_UP: {
      
      general_sequence_timestamp_ = main_system_timer_in_ms_;
      //digitalWrite(configurations_->gpio_pins.h_bridge_pins.steering_wheel.cw_pin, LOW);
      //digitalWrite(configurations_->gpio_pins.h_bridge_pins.steering_wheel.pwm_pin, LOW);
      command(0);
      current_power_up_sequence_stage_ = PowerUpSequenceStages::MOTOR_SHUTDOWN;
      Log.infoln(F("DD Motor has been shutdown, waiting for %dms to turn it back on"), RECOVERY_MOTOR_SHUTDOWN_WAITING_TIME_IN_MS);

      break;
    }
    case PowerUpSequenceStages::MOTOR_SHUTDOWN: {
      if (TimerHelper::timeHasPassed(main_system_timer_in_ms_, general_sequence_timestamp_, RECOVERY_MOTOR_SHUTDOWN_WAITING_TIME_IN_MS))
      {
        //digitalWrite(configurations_->gpio_pins.h_bridge_pins.steering_wheel.cw_pin, HIGH);
        //digitalWrite(configurations_->gpio_pins.h_bridge_pins.steering_wheel.pwm_pin, HIGH);
        current_power_up_sequence_stage_ = PowerUpSequenceStages::MOTOR_DISABLE;
        clearAllMotorFaultCodes();
        general_sequence_timestamp_ = main_system_timer_in_ms_;
        Log.infoln(F("DD Motor power turned on, waiting for %dms to Disable motor"), RECOVERY_MOTOR_DISABLE_WAITING_TIME_IN_MS);
      }
      break;
    }
    case PowerUpSequenceStages::MOTOR_DISABLE: {
      if (TimerHelper::timeHasPassed(main_system_timer_in_ms_, general_sequence_timestamp_, RECOVERY_MOTOR_DISABLE_WAITING_TIME_IN_MS))
      {
        if (enable_motor_once_power_up_completed_)
        {
          current_power_up_sequence_stage_ = PowerUpSequenceStages::MOTOR_ENABLE;
        }
        else {
          current_power_up_sequence_stage_ = PowerUpSequenceStages::POWER_UP_COMPLETED;
        }
        general_sequence_timestamp_ = main_system_timer_in_ms_;
        Log.infoln(F("Disabling DD Motor, waiting for %dms"), RECOVERY_MOTOR_ENABLE_WAITING_TIME_IN_MS);
        sendMotorQuery(disable_motor_payload_data_);
      }
      break;
    }
    case PowerUpSequenceStages::MOTOR_ENABLE: {
      if (TimerHelper::timeHasPassed(main_system_timer_in_ms_, general_sequence_timestamp_, RECOVERY_MOTOR_ENABLE_WAITING_TIME_IN_MS))
      {
        Log.infoln(F("Enabling DD Motor, waiting for %dms to Enable motor"), RECOVERY_MOTOR_COMPLETED_WAITING_TIME_IN_MS);
        current_power_up_sequence_stage_ = PowerUpSequenceStages::POWER_UP_COMPLETED;
        sendMotorQuery(enable_motor_payload_data_);
        command(0);
      }
      break;
    }
    case PowerUpSequenceStages::POWER_UP_COMPLETED: {
      if (TimerHelper::timeHasPassed(main_system_timer_in_ms_, general_sequence_timestamp_, RECOVERY_MOTOR_COMPLETED_WAITING_TIME_IN_MS))
      {
        Log.infoln(F("Motor Power up has been completed !"));
        
        current_power_up_sequence_stage_ = PowerUpSequenceStages::PENDING_POWER_UP_INIT;
        setStableOperationMode();
      }
      break;
    }
    default: {
      break;
    }
  }
}

void DirectDriveSteeringWheel::handleErrorsRaisedByMotor()
{
  if (checkIfMotorFaultCodeIsFlagged(MOTOR_OVER_CURRENT_FAULT_CODE_))
  {
    Log.infoln(F("Self motor Over Current protection !"));
    
    checkIfFaultReachedMaxConsecutive(MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR);
    increaseConsecutiveCounterToFault(MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR);
    initializePowerUpSequence(true);
  }

}

void DirectDriveSteeringWheel::overCurrentContinuousBitProcess()
{
  if (motor_current_in_amps_ > MAX_CURRENT_IN_AMPS)
  {
    Log.infoln(F("LLC Software motor Over Current protection !"));
    
    checkIfFaultReachedMaxConsecutive(MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR);
    increaseConsecutiveCounterToFault(MalfunctionsBitId::DD_STEERING_OVER_CURRENT_ERROR);
    initializePowerUpSequence(true);
  }
}
void DirectDriveSteeringWheel::publishKeepAliveToMotorWhenDisabled()
{
  if (not is_motor_enabled_)
  {
    if (TimerHelper::timeHasPassed(main_system_timer_in_ms_, last_ka_published_to_motor_timestamp, 100)) {
      sendMotorQuery(disable_motor_payload_data_);
    }
  }
}

void DirectDriveSteeringWheel::sendMotorQuery(uint8_t* motor_query_payload_data_)
{
  can_frame can_message_to_send = can_frame();

  can_message_to_send.can_id = MOTOR_COMMANDS_ADDRESS_ID_ | CAN_EFF_FLAG;
  can_message_to_send.can_dlc = 8;

  memcpy(can_message_to_send.data, motor_query_payload_data_, 8);

  mcp2515.sendMessage(&can_message_to_send);
}

const char *DirectDriveSteeringWheel::operationModeToString(MotorOperationMode motor_operation_mode)
{
  const char *stringed_operation =
    motor_operation_mode == MotorOperationMode::POWER_UP_SEQUENCE ? "POWER_UP_SEQUENCE"
    : motor_operation_mode == MotorOperationMode::STABLE ? "STABLE"
    : motor_operation_mode == MotorOperationMode::FAULT ? "FAULT"
    : "UNKNOWN_OPERATION_MODE";

  return stringed_operation;
}

void DirectDriveSteeringWheel::checkForMessageFromMotor()
{
  can_frame response_can_frame{};

  if (mcp2515.readMessage(&response_can_frame) == MCP2515::ERROR_OK)
  {
    switch (response_can_frame.can_id)
    {
      case MOTOR_RESPONSE_ADDRESS_ID_: {
        parseResponseFromMotor(response_can_frame);
        break;
      }
      case MOTOR_HEARTBEAT_ADDRESS_ID_: {
        last_motor_ka_msg_timestamp = main_system_timer_in_ms_;
        parseKeepAliveData(response_can_frame);
        break;
      }
      default: return;
    }
  }
}

void DirectDriveSteeringWheel::printResponseFromMotor(const can_frame &response_can_frame) const
{
  Serial.print(response_can_frame.can_id, HEX);
  Serial.print(" ");
  for (int i = 0; i < response_can_frame.can_dlc; i++)
  {// print the data
    Serial.print(response_can_frame.data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
