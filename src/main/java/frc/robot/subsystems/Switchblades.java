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

  private boolean isTheLeftHomed;
  private boolean isTheRightHomed;

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

    _rightSwitchblade = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_TALON);
    _rightSwitchblade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_RIGHT_ADDRESS, 0);
    _rightSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _rightSwitchblade.setInverted(true);

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
      _leftSwitchblade.set(ControlMode.PercentOutput, -.15);
      isTheLeftHomed = false;
    }

    if (_rightSwitchblade.getSensorCollection().isRevLimitSwitchClosed() == false) {
      _rightSwitchblade.setSelectedSensorPosition(0, RobotMap.ENCODER_RIGHT_ADDRESS, 0);
      _rightSwitchblade.set(ControlMode.PercentOutput, 0);
      isTheRightHomed = true;
    }

    else {
      _rightSwitchblade.set(ControlMode.PercentOutput, -.15);
      isTheRightHomed = false;
    }
  }

  private double outputLeftSensorData() {
   return _leftSwitchblade.getSelectedSensorPosition(RobotMap.ENCODER_LEFT_ADDRESS);
  }

  private double outputRightSensorDate() {
    return _rightSwitchblade.getSelectedSensorPosition(RobotMap.ENCODER_RIGHT_ADDRESS);
  }

  public boolean areTheArmsHomed() {

    if (isTheLeftHomed && isTheRightHomed) {
      return true;
    }

    else {
      return false;
    }
  }

  public void updateDashboard() {
    SmartDashboard.putBoolean("Arms Are Homed", areTheArmsHomed());
    SmartDashboard.putNumber("Left Sensor Position", outputLeftSensorData());
    SmartDashboard.putNumber("RIght Sensor Position", outputRightSensorDate());
  }

}
