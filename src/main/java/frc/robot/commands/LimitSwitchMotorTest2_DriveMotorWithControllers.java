/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.LimitSwitchMotorTest2;
import frc.robot.util.BeakXboxController.Thumbstick;

/**
 * An example command.  You can replace me with your own command.
 */
public class LimitSwitchMotorTest2_DriveMotorWithControllers extends Command 
{
  LimitSwitchMotorTest2 _limitSwitchMotorTest2;
  Thumbstick _leftStick;

  public LimitSwitchMotorTest2_DriveMotorWithControllers(Thumbstick leftStick) 
  {
    _limitSwitchMotorTest2 = LimitSwitchMotorTest2.getInstance();
    requires(_limitSwitchMotorTest2);
    _leftStick = leftStick;

    //requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    _limitSwitchMotorTest2.setMotorSpeed(_leftStick.getY());
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
