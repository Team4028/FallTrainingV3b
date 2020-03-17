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
import frc.robot.RobotMap;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;

/**
 * An example subsystem.  Use this as a template.
 */
public class LimitSwitchTestMotor extends Subsystem implements ISubsystem {
    // define class level working variables

    private DigitalInput _limitSwitch = new DigitalInput(RobotMap.TEST_LIMIT_SWITCH_DIO_PORT);
    private DigitalInput _secondLimitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_NUMERO_DOS);

    private TalonSRX _motor = new TalonSRX(RobotMap.ELEVATOR_LIFT_MASTER_CAN_ADDRESS);

    private boolean lastLimitSwitchValue = false;
    private int currentDirection = 1;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LimitSwitchTestMotor _instance = new LimitSwitchTestMotor();
	
	public static LimitSwitchTestMotor getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
    private LimitSwitchTestMotor() {

  
    }

    //=====================================================================================
	// Public Methods
	//=====================================================================================
  
    @Override 
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
  
        // setDefaultCommand(new MySpecialCommand());
    }

    //=====================================================================================
	// Special Methods for ISubsystem
	//=====================================================================================
    @Override
    public void updateLogData(LogDataBE logData) {
		//logData.AddData("Carriage: LimitSwitch", String.valueOf(get_isCubeInCarriage()));
    }

    @Override
    public void updateDashboard() {
		//SmartDashboard.putString("State: Carriage", get_carriageWheelsState().toString());
    }

    private boolean getLimitSwitchValue() { 
        return _limitSwitch.get(); 
    } 
    
    public void setMotorSpin(double speed) { 
        _motor.set(ControlMode.PercentOutput, speed); 
    } 
    /*
    public void runMotor() { 

        if (!lastLimitSwitchValue && getLimitSwitchValue()) {
            currentDirection *= -1;
        }

        setMotorSpin(currentDirection * 0.1);
        
        lastLimitSwitchValue = getLimitSwitchValue();
    }*/

    public void runMotorv2() {

        if (getLimitSwitchValue()) {
            currentDirection = 1;
        } else if (!_secondLimitSwitch.get()) {
            currentDirection = -1;
        } 

        setMotorSpin(currentDirection * 0.1);
    }
    
}
