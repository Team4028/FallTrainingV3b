/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Infeed;


/**
 * An example command.  You can replace me with your own command.
 */
public class Infeed_ZeroArmEncoders extends Command 
{

    Infeed _infeed = Infeed.getInstance();
  private boolean _isOverrideEnabled;
  public Infeed_ZeroArmEncoders(boolean isOverrideEnabled) 
  {
    _isOverrideEnabled = isOverrideEnabled;
    setInterruptible(false);
    requires(_infeed);

    }
    //requires(Robot.m_subsystem);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (_isOverrideEnabled){
      _infeed.resetAreEncodersZeroed();
      System.out.println("running if (isOverrideEnabled) statement in zero command");
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    System.out.println("running ZeroArmEncoders command");
    _infeed.zeroArmEncoders();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
   return _infeed.getBothArmEncodersZeroed();
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
