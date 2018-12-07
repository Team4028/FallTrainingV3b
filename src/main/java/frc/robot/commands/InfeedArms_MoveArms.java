package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.InfeedArms;

/**
 * An example command.  You can replace me with your own command.
 */
public class InfeedArms_MoveArms extends Command 
{
    InfeedArms _infeedArms = InfeedArms.getInstance();
    private double _angleValue;

  public InfeedArms_MoveArms(double angle) 
  {
    requires(_infeedArms);
    _angleValue = angle;
    System.out.println(angle);
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
      _infeedArms.moveInfeedArms(_angleValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {
  }
}
