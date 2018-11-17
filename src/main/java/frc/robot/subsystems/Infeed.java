/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;


/**
 * An example subsystem.  Use this as a template.
 */
public class Infeed extends Subsystem implements ISubsystem 
{

  // define class level working variables
  private static final double DEGREES_TO_NATIVE_UNITS_CONVERSION = (4096/360);
  private TalonSRX _leftArmMotor;
  private boolean _isLeftArmEncoderZeroed = false;
  private TalonSRX _rightArmMotor;
  private boolean _isRightArmEncoderZeroed = false;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Infeed _instance = new Infeed();
	public static Infeed getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private Infeed()
  {
    _leftArmMotor = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
    _leftArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _isLeftArmEncoderZeroed = false;

    _rightArmMotor = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
    _rightArmMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _isRightArmEncoderZeroed = false;
   // _encoderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
   // _encoderMotor.setSensorPhase(true);
  }

  //=====================================================================================
	// Public Methods
    //=====================================================================================

    public void zeroLeftArmEncoder()
    {
      System.out.println("running zeroLeftArmEncoder Method");
      if (_leftArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false)
      {
        _isLeftArmEncoderZeroed = true;
      _leftArmMotor.setSelectedSensorPosition(0, 0, 0);
      _leftArmMotor.set(ControlMode.PercentOutput, 0);
      }
      else if (_isLeftArmEncoderZeroed == false)
      {
        _leftArmMotor.set(ControlMode.PercentOutput, -.3);
      }
    }
    public void zeroRightArmEncoder(){
      System.out.println("running zeroRightArmEncoder Method");
      if (_rightArmMotor.getSensorCollection().isRevLimitSwitchClosed() == false)
      {
        _isRightArmEncoderZeroed = true;
      _rightArmMotor.setSelectedSensorPosition(0, 0, 0);
      _rightArmMotor.set(ControlMode.PercentOutput, 0);
      }
      else if (_isRightArmEncoderZeroed == false)
      {
        _rightArmMotor.set(ControlMode.PercentOutput, -.3);
      }
    }
    public boolean getLeftArmEncoderZeroed(){
      return _isLeftArmEncoderZeroed;
    }
    public boolean getRightArmEncoderZeroed(){
      return _isRightArmEncoderZeroed;
    }

    public void sendMotorToEncoderPosition(int position){
      if (getLeftArmEncoderZeroed() == true){
        if (getLeftArmEncoderPosition() < position){
          _leftArmMotor.set(ControlMode.PercentOutput, -.1);
        }
        else {
          _leftArmMotor.set(ControlMode.PercentOutput, 0);
        }
      }
    }

    public int getLeftArmEncoderPosition() {
      return _leftArmMotor.getSelectedSensorPosition(0);
    }
    public int getRightArmEncoderPosition(){
      return _rightArmMotor.getSelectedSensorPosition(0);
    }
    public double nativeUnitsToDegrees(int units) {
      return units / DEGREES_TO_NATIVE_UNITS_CONVERSION;
    }
    public double degreesToNativeUnits(int degrees){
      return degrees *DEGREES_TO_NATIVE_UNITS_CONVERSION;
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
    SmartDashboard.putBoolean("Infeed:LeftArmEncoderZeroed", getLeftArmEncoderZeroed());
    SmartDashboard.putNumber("Infeed:LeftArmEncoderPositionDegrees", nativeUnitsToDegrees(getLeftArmEncoderPosition()));
    SmartDashboard.putBoolean("Infeed:RightArmEncoderZeroed", getRightArmEncoderZeroed());
    SmartDashboard.putNumber("Infeed:RightArmEncoderPositionDegrees", nativeUnitsToDegrees(getRightArmEncoderPosition()));
  }
}
