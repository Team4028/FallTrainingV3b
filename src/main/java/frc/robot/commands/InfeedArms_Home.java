package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.InfeedArms;

/**
 * An example command.  You can replace me with your own command.
 */
public class InfeedArms_Home extends Command 
{
    InfeedArms _infeedArms = InfeedArms.getInstance();
    private boolean _isResetHomeMode = false;

  public InfeedArms_Home(boolean isResetHomeMode) 
  {
    requires(_infeedArms);
    _isResetHomeMode = isResetHomeMode;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    if (_isResetHomeMode)
    {
      _infeedArms.resetHomingMethod();
    }
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    _infeedArms.homingMechanism();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return _infeedArms.get_isHomingComplete();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}