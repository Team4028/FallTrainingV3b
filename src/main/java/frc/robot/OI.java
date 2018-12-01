/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.Chassis_DriveWithControllers;
import frc.robot.commands.Switchblades_Home;
import frc.robot.commands.Switchblades_MoveTo180Degrees;
import frc.robot.commands.Switchblades_MoveTo45Degrees;
import frc.robot.commands.Switchblades_MoveTo90Degrees;
import frc.robot.util.BeakXboxController;

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
		_driverGamePad.leftStick.whileActive(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		_driverGamePad.leftStick.whenReleased(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		_driverGamePad.rightStick.whileActive(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		_driverGamePad.rightStick.whenReleased(new Chassis_DriveWithControllers(_driverGamePad.leftStick, _driverGamePad.rightStick));
		_driverGamePad.b.whenPressed(new Switchblades_Home());
		
		_driverGamePad.a.whenPressed(new Switchblades_MoveTo90Degrees());
		_driverGamePad.x.whenPressed(new Switchblades_MoveTo45Degrees());
		_driverGamePad.y.whenPressed(new Switchblades_MoveTo180Degrees());
		  _operatorGamepad = new BeakXboxController(RobotMap.DRIVERS_STATION_OPERATOR_GAMEPAD_USB_PORT);
	}
}
