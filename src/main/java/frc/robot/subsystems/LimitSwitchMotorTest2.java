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

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LimitSwitchMotorTest2 _instance = new LimitSwitchMotorTest2();
	
	public static LimitSwitchMotorTest2 getInstance() {
		return _instance;
    }
	
	// private constructor for singleton pattern
  private LimitSwitchMotorTest2()
  {
    _motor = new TalonSRX(RobotMap.MOTOR_CONTROLLED_BY_LIMIT_SWITCH);
    _motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
  }

  //=====================================================================================
	// Public Methods
    //=====================================================================================
    public void setMotorSpeed(double driveSpeed)
    {
       _motor.set(ControlMode.PercentOutput, driveSpeed);
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
