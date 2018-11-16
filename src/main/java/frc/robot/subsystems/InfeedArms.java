package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;
import frc.robot.RobotMap;


public class InfeedArms extends Subsystem implements ISubsystem  {
 // define class level working variables

    private TalonSRX motor;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static InfeedArms _instance = new InfeedArms();
	
	public static InfeedArms getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private InfeedArms() {
    motor = new TalonSRX(RobotMap.INFEED_ARMS);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    this.zeroEncoder();
    //motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
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
		SmartDashboard.putNumber("Encoder Pulses", getEncounterPulses());
  }

  public void setMotorSpeed(double speed) {
      
      if (motor.getSensorCollection().isRevLimitSwitchClosed()) {
          motor.set(ControlMode.PercentOutput, -speed);
      } else {
          motor.set(ControlMode.PercentOutput, speed);
      }
  }

  public int getEncounterPulses() {

    return motor.getSelectedSensorPosition(0);

  }

  public void zeroEncoder() {
    motor.setSelectedSensorPosition(0, 0, 0);
  }

}
