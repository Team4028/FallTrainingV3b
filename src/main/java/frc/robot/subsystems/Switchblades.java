/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Switchblades extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Switchblades _instance = new Switchblades();

  public boolean isTheLeftHomed;
  public boolean isTheRightHomed;
  private double leftArmUnitsConverted;
  private double rightArmUnitsConverted;

  public static Switchblades getInstance() {
    return _instance;
  }

  public TalonSRX _leftSwitchblade;
  public TalonSRX _rightSwitchblade;

  private Switchblades() {

    _leftSwitchblade = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_TALON);
    _leftSwitchblade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_LEFT_ADDRESS, 0);
    _leftSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _leftSwitchblade.setInverted(false);
    _leftSwitchblade.config_kF(0, 0.3354098361, 0);
    _leftSwitchblade.config_kP(0, 1.5, 0);
    _leftSwitchblade.config_kI(0, 0, 0);
    _leftSwitchblade.config_kD(0, 0, 0);
    _leftSwitchblade.configMotionAcceleration(2000, 0);
    _leftSwitchblade.configMotionCruiseVelocity(3000, 0);

    _rightSwitchblade = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_TALON);
    _rightSwitchblade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_RIGHT_ADDRESS, 0);
    _rightSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _rightSwitchblade.setInverted(true);
    _rightSwitchblade.config_kF(0, 0.3354098361, 0);
    _rightSwitchblade.config_kP(0, 1.5, 0);
    _rightSwitchblade.config_kI(0, 0, 0);
    _rightSwitchblade.config_kD(0, 0, 0);
    _rightSwitchblade.configMotionAcceleration(2000, 0);
    _rightSwitchblade.configMotionCruiseVelocity(3000, 0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void zeroSensors() {

    if (_leftSwitchblade.getSensorCollection().isRevLimitSwitchClosed() == false) {
      _leftSwitchblade.setSelectedSensorPosition(0, RobotMap.ENCODER_LEFT_ADDRESS, 0);
      _leftSwitchblade.set(ControlMode.PercentOutput, 0);
      isTheLeftHomed = true;
    }

    else {
      if (isTheLeftHomed == false) {
        _leftSwitchblade.set(ControlMode.PercentOutput, -.1);
      }
    }
    
    if (_rightSwitchblade.getSensorCollection().isRevLimitSwitchClosed() == false) {
      _rightSwitchblade.setSelectedSensorPosition(0, RobotMap.ENCODER_RIGHT_ADDRESS, 0);
      _rightSwitchblade.set(ControlMode.PercentOutput, 0);
      isTheRightHomed = true;
    }

    else {
      if (isTheRightHomed == false) {
        _rightSwitchblade.set(ControlMode.PercentOutput, -.1);
      }
    }
  }
  
  private double outputLeftSensorData() {
   return ((_leftSwitchblade.getSelectedSensorPosition(0)/4096.0)*360);
  }

  private double outputRightSensorDate() {
    return ((_rightSwitchblade.getSelectedSensorPosition(0)/4096.0)*360);
  }

  public void updateDashboard() {
    SmartDashboard.putBoolean("IsTheLeftHomed", isTheLeftHomed);
    SmartDashboard.putBoolean("IsTheRightHomed", isTheRightHomed);
    SmartDashboard.putNumber("Left Sensor Position", outputLeftSensorData());
    SmartDashboard.putNumber("RIght Sensor Position", outputRightSensorDate());
  }

  public boolean armsHaveHomedAlready() {

    if (isTheLeftHomed && isTheRightHomed) {
      return true;
    }

    else {
      return false;
    }
  }

  public void moveArms90Degrees() {
    _leftSwitchblade.set(ControlMode.MotionMagic, 1024);
    _rightSwitchblade.set(ControlMode.MotionMagic, 1024);
  }

  public void moveArms45Degrees() {
    _leftSwitchblade.set(ControlMode.MotionMagic, 512);
    _rightSwitchblade.set(ControlMode.MotionMagic, 512);
  }

  public void moveArms180Degrees() {
    _leftSwitchblade.set(ControlMode.MotionMagic, 2048);
    _rightSwitchblade.set(ControlMode.MotionMagic, 2048);
  }

  public void resetHomed() {
    isTheLeftHomed = false;
    isTheRightHomed = false;
  }
}
