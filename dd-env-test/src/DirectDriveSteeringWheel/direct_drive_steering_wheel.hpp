//
// Created by daniel-bw on 7/26/23.
//

#ifndef DIRECT_DRIVE_STEERING_WHEEL_HPP_
#define DIRECT_DRIVE_STEERING_WHEEL_HPP_

#include <Arduino.h>
#include <ArduinoLog.h>
#include <mcp2515.h>
#include "../SingletonTimer/singleton_timer.hpp"

class DirectDriveSteeringWheel
{
private:
  enum MalfunctionsBitId
  {
    DD_STEERING_OVER_CURRENT_ERROR = 1UL << 9,
    DD_STEERING_UNRESPONSIVE_ERROR = 1UL << 10,
    DD_STEERING_POWER_UP_SEQUENCE = 1UL << 11,
    DD_STEERING_INTEGRITY_TEST = 1UL << 12,
    DD_STEERING_NO_KEEP_ALIVE = 1UL << 13,
    MAX_DD_STEERING_OVER_CURRENT_ERROR = 1 << 20,
    MAX_DD_STEERING_UNRESPONSIVE_ERROR = 1 << 21,
  };
  
  enum PowerUpSequenceStages
  {
    PENDING_POWER_UP_INIT,
    INITIATE_POWER_UP,
    MOTOR_SHUTDOWN,
    MOTOR_DISABLE,
    MOTOR_ENABLE,
    POWER_UP_COMPLETED
  };
public:
  enum MotorOperationMode
  {
    POWER_UP_SEQUENCE,
    STABLE,
    FAULT,
    UNKNOWN_DD_MOTOR_OPERATION_MODE
  };
  
private:
  MCP2515 mcp2515;
  SingletonTimer &timer_;
  
  int rpm_percentage_ = 0;
  
  uint8_t enable_motor_payload_data_[8] = {0x23, 0x0d, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t disable_motor_payload_data_[8] = {0x23, 0x0c, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t speed_command_motor_payload_data_[8] = {0x23, 0x00, 0x20, 0x01, 0xB9, 0xB0, 0xFF, 0xFF};
  uint8_t motor_current_query_payload_data_[8] = {0x40, 0x00, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t motor_voltage_query_payload_data_[8] = {0x40, 0x0D, 0x21, 0x02, 0x00, 0x00, 0x00, 0x00};
  uint8_t motor_temperature_query_payload_data_[8] = {0x40, 0x0F, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t encoder_motor_ang_velocity_query_payload_data_[8] = {0x40, 0x03, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t motor_position_query_payload_data_[8] = {0x40, 0x04, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t fault_query_payload_data_[8] = {0x40, 0x12, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00};
  uint8_t program_version_payload_data_[8] = {0x40, 0x01, 0x11, 0x11, 0x00, 0x00, 0x00, 0x00};

  static constexpr uint32_t MOTOR_ID_ = 0x00000001;
  static constexpr uint32_t MOTOR_COMMANDS_ADDRESS_ID_ = 0x06000000 + MOTOR_ID_;
  static constexpr uint32_t MOTOR_HEARTBEAT_ADDRESS_ID_ = 0x87000000 + MOTOR_ID_;
  static constexpr uint32_t MOTOR_RESPONSE_ADDRESS_ID_ = 0x85800000 + MOTOR_ID_;

  static constexpr uint32_t MOTOR_CURRENT_RESPONSE_PREFIX_ = 0x60002101;
  static constexpr uint32_t MOTOR_VOLTAGE_RESPONSE_PREFIX_ = 0x600D2102;
  static constexpr uint32_t MOTOR_TEMPERATURE_RESPONSE_PREFIX_ = 0x600F2101;
  static constexpr uint32_t MOTOR_ENCODER_VELOCITY_RESPONSE_PREFIX_ = 0x60032101;
  static constexpr uint32_t MOTOR_ENCODER_POSITION_RESPONSE_PREFIX_ = 0x60042101;
  static constexpr uint32_t FAULT_RESPONSE_PREFIX_ = 0x60122101;
  static constexpr uint32_t MOTOR_ENABLED_CONFIRMATION_PREFIX_ = 0x600D2000;
  static constexpr uint32_t MOTOR_DISABLED_CONFIRMATION_PREFIX_ = 0x600C2000;
  static constexpr uint32_t MOTOR_PROGRAM_VERSION_PREFIX_ = 0x60011111;
  
  static constexpr uint16_t MOTOR_DISABLED_FAULT_CODE_ = 0x0001;
  static constexpr uint16_t MOTOR_OVER_CURRENT_FAULT_CODE_ = 0x0040;

  static constexpr int MOTOR_KEEP_ALIVE_TIMEOUT_IN_MS_ = 1000;

  MotorOperationMode motor_operation_mode_ = MotorOperationMode::UNKNOWN_DD_MOTOR_OPERATION_MODE;
  
  unsigned long main_system_timer_in_ms_ = 0;
  unsigned long last_motor_ka_msg_timestamp = 0;
  unsigned long last_ka_published_to_motor_timestamp = 0;
  unsigned long general_sequence_timestamp_ = 0;

  bool is_motor_enabled_ = false;
  int motor_program_version_[4] = {0};

  static constexpr float MAX_CURRENT_IN_AMPS = 20.0;
  float motor_current_in_amps_ = 0.0;
  float motor_voltage_in_volts_ = 0.0;
  short motor_temperature_in_deg_ = 0.0;
  float motor_encoder_velocity_ = 0.0;
  int32_t motor_encoder_position_ = 0;
  
  unsigned long last_ka_timestamp_;
  int32_t last_motor_encoder_position_ = 0;
  
  uint16_t motor_fault_code_ = 0x0000;

  unsigned int consecutive_unresponsive_faults_ = 0;
  unsigned int consecutive_over_current_faults_ = 0;
  static constexpr int MAX_CONSECUTIVE_UNRESPONSIVE_FAULTS = 3;
  static constexpr int MAX_CONSECUTIVE_OVER_CURRENT_FAULTS = 3;
  

  PowerUpSequenceStages current_power_up_sequence_stage_ = PowerUpSequenceStages::PENDING_POWER_UP_INIT;
  static constexpr int RECOVERY_MOTOR_SHUTDOWN_WAITING_TIME_IN_MS = 1000;
  static constexpr int RECOVERY_MOTOR_DISABLE_WAITING_TIME_IN_MS = 3000;
  static constexpr int RECOVERY_MOTOR_ENABLE_WAITING_TIME_IN_MS = 1000;
  static constexpr int RECOVERY_MOTOR_COMPLETED_WAITING_TIME_IN_MS = 500;
  bool motor_has_moved_at_least_once_ = false;
  bool enable_motor_once_power_up_completed_ = false;
  
  bool motor_keep_alive_is_healthy_ = false;
  
public:
  DirectDriveSteeringWheel();

  virtual ~DirectDriveSteeringWheel();

private:
  void initializeCanCommunication();
  
  void checkForMessageFromMotor();
  void parseResponseFromMotor(const can_frame &response_can_frame);
  void parseKeepAliveData(const can_frame &response_can_frame);
  void printResponseFromMotor(const can_frame &response_can_frame) const;

  bool command(int rpm_percentage);
  
  bool changeMotorOperationMode(MotorOperationMode new_motor_operation_mode);
  const char* operationModeToString(MotorOperationMode motor_operation_mode);
  
  void initializePowerUpSequence(bool enable_motor_once_completed = false);
  void powerUpSequenceProcess();
  bool isPowerUpSequenceInProgress();

  bool setStableOperationMode();
  void stableOperationModeProcess();
  
  void resetFaultConsecutiveCounter(MalfunctionsBitId malfunction_id);
  void increaseConsecutiveCounterToFault(MalfunctionsBitId malfunction_id);
  void checkIfFaultReachedMaxConsecutive(MalfunctionsBitId malfunction_id);
  
  void overCurrentContinuousBitProcess();
  void handleErrorsRaisedByMotor();

  bool motorKeepAliveContinuousBuiltInTest();

  void publishKeepAliveToMotorWhenDisabled();
  
  bool isVersionMotorUpdated();

  void sendMotorQuery(uint8_t* data);

  void clearAllMotorFaultCodes() { motor_fault_code_ &= 0x0000; };
  void clearMotorFaultCode(unsigned long motor_fault_code) { motor_fault_code_ &= ~motor_fault_code; };
  void setMotorFaultCode(unsigned long motor_fault_code) { motor_fault_code_ |= motor_fault_code; };
  bool checkIfMotorFaultCodeIsFlagged(uint16_t potential_fault_code) const { return (motor_fault_code_ & potential_fault_code) != 0; };
  bool checkIfMotorDisabledFaultCodeFlagged() { return checkIfMotorFaultCodeIsFlagged(MOTOR_DISABLED_FAULT_CODE_); };

public:
  void setRpmPercentage(int rpm_percentage);
  
public:
  void lockActuator();
  void releaseActuator();

  void tick();
  
  bool performPowerUpBuiltInTest();
  bool performContinuousBuiltInTest();
  
  MotorOperationMode getOperationMode() const { return motor_operation_mode_;};
  
  float getMotorCurrentInAmps() const { return motor_current_in_amps_; }
  float getMotorVoltageInVolts() const { return motor_voltage_in_volts_; }
  short getMotorTemperatureInDegree() const { return motor_temperature_in_deg_; }
  float getMotorEncoderVelocity() const { return motor_encoder_velocity_; }
  int32_t getMotorEncoderPosition() const { return motor_encoder_position_; }
  uint16_t getMotorFaultCode() const { return motor_fault_code_; }
};

#endif
