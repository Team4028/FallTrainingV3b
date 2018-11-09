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
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;

/**
 * An example subsystem.  Use this as a template.
 */
public class EncoderMotorTest extends Subsystem implements ISubsystem 
{
  // define class level working variables
  private TalonSRX _encoderMotor;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static EncoderMotorTest _instance = new EncoderMotorTest();
	public static EncoderMotorTest getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private EncoderMotorTest()
  {
    _encoderMotor = new TalonSRX(RobotMap.MOTOR_CONTROLLED_BY_ENCODER);
    _encoderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    _encoderMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
 
  }

  //=====================================================================================
	// Public Methods
    //=====================================================================================
    public void runMotor()
    {
      _encoderMotor.set(ControlMode.PercentOutput, .1);
      System.out.println(getEncoderPosition());
    }
    public void zeroSensor()
    {
      if (_encoderMotor.getSensorCollection().isRevLimitSwitchClosed() == false)
      {
      _encoderMotor.setSelectedSensorPosition(0, 0, 0);
      }
      else ()
      {
        
      }
    }

    private double getEncoderPosition() 
    {
      return _encoderMotor.getSelectedSensorPosition(0);
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
  }
}
