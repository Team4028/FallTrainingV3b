/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;

/**
 * An example subsystem.  Use this as a template.
 */
public class LimitSwitchMotorTest2 extends Subsystem implements ISubsystem 
{
  // define class level working variables
  private TalonSRX _motor;
  private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096.0 / 360);
  private static final double INFEED_MOTION_MAGIC_F = 0.3354098361;
  private static final double INFEED_MOTION_MAGIC_P = 1.5;
  private static final double INFEED_MOTION_MAGIC_I = 0;
  private static final double INFEED_MOTION_MAGIC_D = 0;
  private static final double MOTOR_SPEED_FOR_ZEROING = -.05;
  private boolean _isMotorEncoderZeroed = false;
  private double allowedDeadbandDegrees = 1.5;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LimitSwitchMotorTest2 _instance = new LimitSwitchMotorTest2();
	
	public static LimitSwitchMotorTest2 getInstance() {
		return _instance;
    }
	
	// private constructor for singleton pattern
  private LimitSwitchMotorTest2(){
    _motor = new TalonSRX(RobotMap.MOTOR_CONTROLLED_BY_LIMIT_SWITCH);
    _motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _motor.config_kD(0, INFEED_MOTION_MAGIC_D, 0);
    _motor.config_kF(0, INFEED_MOTION_MAGIC_F, 0);
    _motor.config_kI(0, INFEED_MOTION_MAGIC_I, 0);
    _motor.config_kP(0, INFEED_MOTION_MAGIC_P, 0);
    _motor.configMotionAcceleration(2000, 0);
    _motor.configMotionCruiseVelocity(3000, 0);
  }


  //=====================================================================================
	// Public Methods
    //=====================================================================================
    public void setMotorSpeed(double driveSpeed)
    {
       _motor.set(ControlMode.PercentOutput, driveSpeed);
    }
    public void zeroMotorEncoder(){
      if (_motor.getSensorCollection().isRevLimitSwitchClosed() == false) {
        _isMotorEncoderZeroed = true;
        _motor.setSelectedSensorPosition(0, 0, 0);
        _motor.set(ControlMode.PercentOutput, 0);
      }
      else if (_isMotorEncoderZeroed == false) {
        _motor.set(ControlMode.PercentOutput, MOTOR_SPEED_FOR_ZEROING);
      }
    }
    public void goToEncoderPosition(double degreesPos){
      double nativeUnitsPos = degreesToNativeUnits(degreesPos);
      _motor.set(ControlMode.MotionMagic, nativeUnitsPos);
    }
    public double getEncoderPosition(){
      return _motor.getSelectedSensorPosition(0);
    }
    public boolean getIsLimitSwitchPressed(){
      return _motor.getSensorCollection().isRevLimitSwitchClosed();
    }
    public double nativeUnitsToDegrees(double units) {
      return units / DEGREES_TO_NATIVE_UNITS_CONVERSION;
    }
  
    public double degreesToNativeUnits(double degrees) {
      return degrees * DEGREES_TO_NATIVE_UNITS_CONVERSION;
    }
    public boolean getIsMotorHomed(){
      return _isMotorEncoderZeroed;
    }
    public void resetIsMotorEncoderZeroed(){
      _isMotorEncoderZeroed = false;
    }
    public boolean getIsMotorAtEncoderPosition(double degreesPos){
      double nativeUnitsPos = degreesToNativeUnits(degreesPos);
      double allowedDeadbandNativeUnits = degreesToNativeUnits(allowedDeadbandDegrees);
      double motorPositionDeadband = Math.abs(nativeUnitsPos - getEncoderPosition());
      if (motorPositionDeadband <= allowedDeadbandNativeUnits){
        return true;
      }
      else {
        return false;
      }
    }

  @Override
  public void initDefaultCommand() 
  {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //=====================================================================================
	// Special Methods for ISubsystem
	//=====================================================================================
  @Override
  public void updateLogData(LogDataBE logData) 
  {
		//logData.AddData("Carriage: LimitSwitch", String.valueOf(get_isCubeInCarriage()));
  }

  @Override
  public void updateDashboard() 
  {
    //SmartDashboard.putString("State: Carriage", get_carriageWheelsState().toString());
    SmartDashboard.putNumber("LimitSwitchMotorTest2:MotorVelocityNativeUnits", _motor.getSelectedSensorVelocity(0));
  }
}
