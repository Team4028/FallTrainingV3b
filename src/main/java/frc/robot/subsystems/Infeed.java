/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;

/**
 * An example subsystem. Use this as a template.
 */
public class Infeed extends Subsystem implements ISubsystem {

  // define class level working variables
  private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096 / 360);
  public static final double ARM_MOTORS_SPEED_FOR_ZEROING = -.15;
  public static final double ARM_MOTORS_SPEED_FOR_GOING_TO_POS = .1;
  private static final double INFEED_MOTION_MAGIC_F = 0.3354098361;
  private static final double INFEED_MOTION_MAGIC_P = 1.5;
  private static final double INFEED_MOTION_MAGIC_I = 0;
  private static final double INFEED_MOTION_MAGIC_D = 0;
  private static final int MAX_A = 2000;
  private static final int MAX_V = 3000;
  private TalonSRX _leftArmMotor;
  private boolean _isLeftArmEncoderZeroed = false;
  private boolean _isLeftArmAtSetPosition = false;

  private TalonSRX _rightArmMotor;
  private boolean _isRightArmEncoderZeroed = false;
  private boolean _isRightArmAtSetPosition = false;

  private double allowedDeadbandDegrees = 2.75;

  // =====================================================================================
  // Define Singleton Pattern
  // =====================================================================================
  private static Infeed _instance = new Infeed();

  public static Infeed getInstance() {
    return _instance;
  }

  // private constructor for singleton pattern
  private Infeed() {
    _leftArmMotor = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
    _leftArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _leftArmMotor.config_kD(0, INFEED_MOTION_MAGIC_D, 0);
    _leftArmMotor.config_kF(0, INFEED_MOTION_MAGIC_F, 0);
    _leftArmMotor.config_kI(0, INFEED_MOTION_MAGIC_I, 0);
    _leftArmMotor.config_kP(0, INFEED_MOTION_MAGIC_P, 0);
    _leftArmMotor.configMotionAcceleration(MAX_A, 0);
    _leftArmMotor.configMotionCruiseVelocity(MAX_V, 0);
    _isLeftArmEncoderZeroed = false;
    _isLeftArmAtSetPosition = false;

    _rightArmMotor = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
    _rightArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _rightArmMotor.config_kD(0, INFEED_MOTION_MAGIC_D, 0);
    _rightArmMotor.config_kF(0, INFEED_MOTION_MAGIC_F, 0);
    _rightArmMotor.config_kI(0, INFEED_MOTION_MAGIC_I, 0);
    _rightArmMotor.config_kP(0, INFEED_MOTION_MAGIC_P, 0);
    _rightArmMotor.configMotionAcceleration(MAX_A, 0);
    _rightArmMotor.configMotionCruiseVelocity(MAX_V, 0);
    _isRightArmEncoderZeroed = false;
    _rightArmMotor.setInverted(true);
    _isRightArmAtSetPosition = false;
    // _encoderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    // 0, 0);
    // _encoderMotor.setSensorPhase(true);
  }

  // =====================================================================================
  // Public Methods
  // =====================================================================================
  public void zeroArmEncoders() {
    if (_leftArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
      _isLeftArmEncoderZeroed = true;
      _leftArmMotor.setSelectedSensorPosition(0, 0, 0);
      _leftArmMotor.set(ControlMode.PercentOutput, 0);
    }    
    else if (_isLeftArmEncoderZeroed == false) {
      _leftArmMotor.set(ControlMode.PercentOutput, ARM_MOTORS_SPEED_FOR_ZEROING);
    }
    if (_rightArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false) {
      _isRightArmEncoderZeroed = true;
      _rightArmMotor.setSelectedSensorPosition(0, 0, 0);
      _rightArmMotor.set(ControlMode.PercentOutput, 0);
    }
    else if (_isRightArmEncoderZeroed == false) {
      _rightArmMotor.set(ControlMode.PercentOutput, ARM_MOTORS_SPEED_FOR_ZEROING);
    }

  }

  public void armsGoToEncoderPos(double degreesPos){
    double nativeUnitsPos = degreesToNativeUnits(degreesPos);
      _rightArmMotor.set(ControlMode.MotionMagic, nativeUnitsPos);
      _leftArmMotor.set(ControlMode.MotionMagic, nativeUnitsPos);
    }

  public boolean getLeftArmEncoderZeroed() {
    return _isLeftArmEncoderZeroed;
  }

  public boolean getRightArmEncoderZeroed() {
    return _isRightArmEncoderZeroed;
  }

  public boolean getBothArmEncodersZeroed() {
    if (getRightArmEncoderZeroed() && getLeftArmEncoderZeroed()) {
      return true;
    } else {
      return false;
    }
  }

  public void resetAreEncodersZeroed() {
    _isLeftArmEncoderZeroed = false;
    _isRightArmEncoderZeroed = false;
  }

  public double getLeftArmEncoderPosition() {
    return _leftArmMotor.getSelectedSensorPosition(0);
  }

  public double getRightArmEncoderPosition() {
    return _rightArmMotor.getSelectedSensorPosition(0);
  }
  public boolean getAreArmsAtEncoderPosition(double degreesPos){
    double nativeUnitsPos = degreesToNativeUnits(degreesPos);
    double allowedDeadbandNativeUnits = degreesToNativeUnits(allowedDeadbandDegrees);
    double rightArmDeadband = Math.abs(nativeUnitsPos - getRightArmEncoderPosition());
    double leftArmDeadband = Math.abs(nativeUnitsPos - getLeftArmEncoderPosition());
    if (rightArmDeadband <= allowedDeadbandNativeUnits && leftArmDeadband <= allowedDeadbandNativeUnits){
      return true;
    }
    else {
      return false;
    }
  }

  public double nativeUnitsToDegrees(double units) {
    return units / DEGREES_TO_NATIVE_UNITS_CONVERSION;
  }

  public double degreesToNativeUnits(double degrees) {
    return degrees * DEGREES_TO_NATIVE_UNITS_CONVERSION;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // =====================================================================================
  // Special Methods for ISubsystem
  // =====================================================================================
  @Override
  public void updateLogData(LogDataBE logData) {
    // logData.AddData("Carriage: LimitSwitch",
    // String.valueOf(get_isCubeInCarriage()));
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putBoolean("Infeed:LeftArmEncoderZeroed", getLeftArmEncoderZeroed());
    SmartDashboard.putNumber("Infeed:LeftArmEncoderPositionDegrees", nativeUnitsToDegrees(getLeftArmEncoderPosition()));
    SmartDashboard.putNumber("Infeed:LeftArmEncoderPositionNativeUnits", getLeftArmEncoderPosition());
    SmartDashboard.putBoolean("Infeed:RightArmEncoderZeroed", getRightArmEncoderZeroed());
    SmartDashboard.putNumber("Infeed:RightArmEncoderPositionDegrees",
        nativeUnitsToDegrees(getRightArmEncoderPosition()));
    SmartDashboard.putNumber("Infeed:RightArmEncoderPositionNativeUnits", getRightArmEncoderPosition());
  }
}
