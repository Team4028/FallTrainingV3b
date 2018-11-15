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
public class Encoder_Thing extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Encoder_Thing _instance = new Encoder_Thing();

  public static Encoder_Thing getInstance() {

    return _instance;
  
  }

  TalonSRX _encoderTalon;

  private Encoder_Thing() {

    _encoderTalon = new TalonSRX(RobotMap.TEST_TALON);

    _encoderTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_ADDRESS, 0);

    _encoderTalon.configReverseLimitSwitchSource(LimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyClosed, 0);

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

    if (_encoderTalon.getSensorCollection().isRevLimitSwitchClosed() == false) {
     
      _encoderTalon.setSelectedSensorPosition(0, RobotMap.ENCODER_ADDRESS, 0);
     
      _encoderTalon.set(ControlMode.PercentOutput, 0);
    
    }

    else {

      _encoderTalon.set(ControlMode.PercentOutput, -.25);
      
    }
  }
}
