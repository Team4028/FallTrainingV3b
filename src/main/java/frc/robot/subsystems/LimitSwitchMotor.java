/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;

/**
 * An example subsystem.  Use this as a template.
 */
public class LimitSwitchMotor extends Subsystem implements ISubsystem 
{
  // define class level working variables
  private TalonSRX _motor;
  public DigitalInput _limitSwitch;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LimitSwitchMotor _instance = new LimitSwitchMotor();
	
	public static LimitSwitchMotor getInstance() {
		return _instance;
    }
    public boolean getLimitSwitchValue()
    {
       return _limitSwitch.get();
    }
	
	// private constructor for singleton pattern
  private LimitSwitchMotor()
  {
    _motor = new TalonSRX(RobotMap.ELEVATOR_LIFT_MASTER_CAN_ADDRESS);
    _limitSwitch = new DigitalInput(RobotMap.TEST_LIMIT_SWITCH_DIO_PORT);

  }

  //=====================================================================================
	// Public Methods
    //=====================================================================================
    public void controlMotorWithLimitSwitch()
    {
        if (getLimitSwitchValue() == false)
        {
            _motor.set(ControlMode.PercentOutput, 0.1);
            
        }
        else if (getLimitSwitchValue() == true)
        {
            _motor.set(ControlMode.PercentOutput, -0.1);
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
  }
}
