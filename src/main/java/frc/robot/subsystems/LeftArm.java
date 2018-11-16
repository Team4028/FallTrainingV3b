/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LeftArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LeftArm _instance = new LeftArm();

  public static LeftArm getInstance() {

    return _instance;
  
  }

  public TalonSRX _encoderTalon;

  private LeftArm() {

    _encoderTalon = new TalonSRX(RobotMap.TEST_TALON);

    _encoderTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_ADDRESS, 0);

    _encoderTalon.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, 0);

    _encoderTalon.setInverted(true);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  private double EncoderPosition() {

    return _encoderTalon.getSelectedSensorPosition(RobotMap.ENCODER_ADDRESS);

  }

  public void runMotor() {

    _encoderTalon.set(ControlMode.PercentOutput, .25);

    System.out.println(EncoderPosition());
      
   
  }

  public void zeroSensor() {

    _encoderTalon.set(ControlMode.PercentOutput, -.1);
      
  }

  public boolean isTheLimitSwitchClosed() {
  
    if (_encoderTalon.getSensorCollection().isFwdLimitSwitchClosed() == false) {

      return true;

    }

    else {

      return false;

    }

  }

}
