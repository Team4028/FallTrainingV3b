/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.Carriage_Home;
import frc.robot.commands.Chassis_DriveWithControllers;
import frc.robot.commands.Chassis_ShiftGear;
import frc.robot.commands.InfeedArms_DriveUntil;
import frc.robot.commands.LimitSwitchMotor_Drive;
import frc.robot.util.BeakXboxController;
import frc.robot.commands.InfeedArms_DriveUntil;
import frc.robot.commands.Carriage_Rehome;
import frc.robot.commands.Carriage_SetPosition;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI 
{
  // working object
  BeakXboxController _driverGamePad;
	BeakXboxController _operatorGamepad;
	

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static OI _instance = new OI();
	
	public static OI getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private OI() {
		_driverGamePad = new BeakXboxController(RobotMap.DRIVERS_STATION_DRIVER_GAMEPAD_USB_PORT);
		//_driverGamePad.leftStick.whileActive(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		//_driverGamePad.leftStick.whenReleased(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		//_driverGamePad.rightStick.whileActive(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		//_driverGamePad.rightStick.whenReleased(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		//_driverGamePad.b.whenPressed(new Chassis_ShiftGear());
		//_driverGamePad.a.whenPressed(new LimitSwitchMotor_Drive());
    	_operatorGamepad = new BeakXboxController(RobotMap.DRIVERS_STATION_OPERATOR_GAMEPAD_USB_PORT);
		//_driverGamePad.x.whenPressed(new InfeedArms_DriveUntil());
		_driverGamePad.a.whenPressed(new Carriage_Rehome());
		_driverGamePad.b.whenPressed(new Carriage_SetPosition(90.0));
		_driverGamePad.x.whenPressed(new Carriage_SetPosition(45.0));
		_driverGamePad.y.whenPressed(new Carriage_SetPosition(180.0));
	}
}
