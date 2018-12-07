package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;

/**
 * An example subsystem.  Use this as a template.
 */
public class InfeedArms extends Subsystem implements ISubsystem
{
    private TalonSRX _rightSwitchblade;
    private TalonSRX _leftSwitchblade;
    private boolean _isRightSwitchbladeHomed;
    private boolean _isLeftSwitchbladeHomed;
    private static int MOTION_MAGIC_MAX_V = 3000;
    private static int MOTION_MAGIC_MAX_A = 2000;
    private static double TALON_FEED_FORWARD_GAIN = 0.3354098361;
    private static double TALON_PROPORTIONAL_GAIN = 1.5;
    private static double TALON_INTEGRAL_GAIN = 0;
    private static double TALON_DERIVATIVE_GAIN = 0;

    private static InfeedArms _instance = new InfeedArms();

    public static InfeedArms getInstance()
    {
        return _instance;
    }


    private InfeedArms()
    {
        _rightSwitchblade = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
        _rightSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
        _rightSwitchblade.setInverted(true);
        _rightSwitchblade.config_kF(0, TALON_FEED_FORWARD_GAIN, 0);
        _rightSwitchblade.config_kP(0, TALON_PROPORTIONAL_GAIN, 0);
        _rightSwitchblade.config_kI(0, TALON_INTEGRAL_GAIN, 0);
        _rightSwitchblade.config_kD(0, TALON_DERIVATIVE_GAIN, 0);
        _rightSwitchblade.configMotionAcceleration(MOTION_MAGIC_MAX_A, 0);
        _rightSwitchblade.configMotionCruiseVelocity(MOTION_MAGIC_MAX_V, 0);

        _leftSwitchblade = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
        _leftSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
        _leftSwitchblade.setInverted(false);
        _leftSwitchblade.config_kF(0, TALON_FEED_FORWARD_GAIN, 0);
        _leftSwitchblade.config_kP(0, TALON_PROPORTIONAL_GAIN, 0);
        _leftSwitchblade.config_kI(0, TALON_INTEGRAL_GAIN, 0);
        _leftSwitchblade.config_kD(0, TALON_DERIVATIVE_GAIN, 0);
        _leftSwitchblade.configMotionAcceleration(MOTION_MAGIC_MAX_A, 0);
        _leftSwitchblade.configMotionCruiseVelocity(MOTION_MAGIC_MAX_V, 0);
    }

    public void homingMechanism()
    {
        if(!_rightSwitchblade.getSensorCollection().isRevLimitSwitchClosed())
        {
            _rightSwitchblade.set(ControlMode.PercentOutput, 0);
            _rightSwitchblade.setSelectedSensorPosition(0, 0, 0);
            _isRightSwitchbladeHomed = true;
        }
        else if (!_isRightSwitchbladeHomed)
        {
            _rightSwitchblade.set(ControlMode.PercentOutput, -.15);
        }

        if(!_leftSwitchblade.getSensorCollection().isRevLimitSwitchClosed())
        {
            _leftSwitchblade.set(ControlMode.PercentOutput, 0);
            _leftSwitchblade.setSelectedSensorPosition(0, 0, 0);
            _isLeftSwitchbladeHomed = true;
        }
        else if (!_isLeftSwitchbladeHomed)
        {
            _leftSwitchblade.set(ControlMode.PercentOutput, -.15);
        }
    }

    public void resetHomingMethod()
    {
        _isLeftSwitchbladeHomed = false;
        _isRightSwitchbladeHomed = false;
    }

    @Override
    public void updateLogData(LogDataBE logData) 
     {
		//logData.AddData("Carriage: LimitSwitch", String.valueOf(get_isCubeInCarriage()));
    }

    public boolean get_isHomingComplete()
    {
        return _isLeftSwitchbladeHomed && _isRightSwitchbladeHomed;
    }

    @Override
     public void updateDashboard() 
    {
          SmartDashboard.putNumber("InfeedArms: Right Arm Position nu:", _rightSwitchblade.getSelectedSensorPosition(0));
          SmartDashboard.putNumber("InfeedArms: Left Arm Position nu:", _leftSwitchblade.getSelectedSensorPosition(0));
          SmartDashboard.putBoolean("InfeedArms: Left arm homed?", _isLeftSwitchbladeHomed);
          SmartDashboard.putBoolean("InfeedArms: Right arm homed?", _isRightSwitchbladeHomed);
          SmartDashboard.putNumber("InfeedArms: Right Arm Position deg:", _rightSwitchblade.getSelectedSensorPosition(0)*(360/4096));
          SmartDashboard.putNumber("InfeedArms: Left Arm Position deg:", _leftSwitchblade.getSelectedSensorPosition(0)*(360/4096));
    }

    public void moveInfeedArms(double angle)
    {
        //System.out.println(angle);
        _rightSwitchblade.set(ControlMode.MotionMagic, angle/(360.0/4096.0));
        _leftSwitchblade.set(ControlMode.MotionMagic, angle/(360.0/4096.0));
    }

    @Override
    protected void initDefaultCommand() 
    {
    }
}