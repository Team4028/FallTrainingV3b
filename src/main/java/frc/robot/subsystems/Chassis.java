/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;

/**
 * An example subsystem.  Use this as a template.
 */
public class Chassis extends Subsystem implements ISubsystem 
{
  // define class level working variables
  private TalonSRX _leftMasterMotor;
  private TalonSRX _rightMasterMotor;
  private TalonSRX _rightSlaveMotor;
  private TalonSRX _leftSlaveMotor;
  private DoubleSolenoid _gearShifter;
  private static final Value SHIFTER_LOW_GEAR_POS = DoubleSolenoid.Value.kReverse;
  private static final Value SHIFTER_HIGH_GEAR_POS = DoubleSolenoid.Value.kForward;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private Chassis()
  {
    _leftMasterMotor = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
    _leftSlaveMotor = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
    _leftSlaveMotor.follow(_leftMasterMotor);

    _rightMasterMotor = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
    _rightSlaveMotor = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);
    _rightSlaveMotor.follow(_rightMasterMotor);

    _gearShifter = new DoubleSolenoid(RobotMap.SHIFTER_EXTEND_PCM_PORT, RobotMap.SHIFTER_RETRACT_PCM_PORT);
  }

  //=====================================================================================
	// Public Methods
    //=====================================================================================
    
    public void setMotorSpeed (double driveSpeed, double turnSpeed)
    {
        double leftSpeed = (.7 * -driveSpeed) + (.5 * -turnSpeed);
        double rightSpeed = (.7 * driveSpeed) + (.5 * -turnSpeed);
        //set the speed for the right chassis motors
        _rightMasterMotor.set(ControlMode.PercentOutput, rightSpeed);
        


        //set the speed for the left chassis motors
        _leftMasterMotor.set(ControlMode.PercentOutput, leftSpeed);

    }
    public void toggleGearShift()
    {
        //toggle the gear shift
        Value currentGear = _gearShifter.get();
        if(currentGear == SHIFTER_HIGH_GEAR_POS)
        {
            _gearShifter.set(SHIFTER_LOW_GEAR_POS);
        }
        else
        {
            _gearShifter.set(SHIFTER_HIGH_GEAR_POS);
        }

    }
    public synchronized boolean highGear()
    {
        return _gearShifter.get() == SHIFTER_HIGH_GEAR_POS;
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
