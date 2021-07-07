package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.LimitSwitchMotor;
import frc.robot.util.BeakXboxController.Thumbstick;

/**
 * An example command.  You can replace me with your own command.
 */
public class Chassis_ShiftGear extends Command {

    Chassis _chassis = Chassis.getInstance();

  public Chassis_ShiftGear()
  {
    requires(_chassis);
    //requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize()
  {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute()
  {
      _chassis.toggleGearShift();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
