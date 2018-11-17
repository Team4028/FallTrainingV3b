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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class LeftArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static LeftArm _instance = new LeftArm();

  public boolean areTheArmsHomed;

  public static LeftArm getInstance() {

    return _instance;
  
  }

  public TalonSRX _encoderTalon;

  private LeftArm() {

    _encoderTalon = new TalonSRX(RobotMap.LEFT_ARM_TALON);

    _encoderTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.ENCODER_ADDRESS, 0);

    _encoderTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

    _encoderTalon.setInverted(false);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void zeroSensor() {

    if (_encoderTalon.getSensorCollection().isRevLimitSwitchClosed() == false) {

      _encoderTalon.setSelectedSensorPosition(0, RobotMap.ENCODER_ADDRESS, 0);

      _encoderTalon.set(ControlMode.PercentOutput, 0);

      areTheArmsHomed = true;

    }

    else {
    
      _encoderTalon.set(ControlMode.PercentOutput, -.15);

      areTheArmsHomed = false;
    
    }

    /*if (_encoderTalon.getSensorCollection().isRevLimitSwitchClosed()) {

      areTheFrickinArmsHomed = false;

    }

    else {

      areTheFrickinArmsHomed = true;

    }*/

  }

  public double outputSensorData() {

   return _encoderTalon.getSelectedSensorPosition(RobotMap.ENCODER_ADDRESS);

  }

  public void updateDashboard() {

    SmartDashboard.putBoolean("Arms Are Homed", areTheArmsHomed);
		SmartDashboard.putNumber("Sensor Position", outputSensorData());
  
  }

}
