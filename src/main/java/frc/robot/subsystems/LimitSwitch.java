/*----------------------------------------------------------------------------
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             
/* Open Source Software - may be modified and shared by FRC teams. The code   
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.

public class LimitSwitch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LimitSwitch _instance = new LimitSwitch();

  public static LimitSwitch getInstance() {
    
    return _instance;
  
  }

  private TalonSRX _talon = new TalonSRX(RobotMap.TEST_TALON);

  private LimitSwitch() {

    _talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

  }


  public void runMotor() {

    _talon.set(ControlMode.PercentOutput, -.5);

  }

  public void stopMotor() {

    _talon.set(ControlMode.PercentOutput, 0);

  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
*/
