/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.LimitSwitchMotorTest2;

/**
 * An example command.  You can replace me with your own command.
 */
public class LimitSwitchMotorTest2_ZeroEncoder extends Command 
{
  LimitSwitchMotorTest2 _limitSwitchMotorTest2;
  private boolean _isOverrideEnabled;

  public LimitSwitchMotorTest2_ZeroEncoder(boolean isOverrideEnabled) 
  {
    _limitSwitchMotorTest2 = LimitSwitchMotorTest2.getInstance();
    requires(_limitSwitchMotorTest2);
    _isOverrideEnabled = isOverrideEnabled;

    //requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      if(_isOverrideEnabled){
          _limitSwitchMotorTest2.resetIsMotorEncoderZeroed();
      }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    _limitSwitchMotorTest2.zeroMotorEncoder();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _limitSwitchMotorTest2.getIsMotorHomed();
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
