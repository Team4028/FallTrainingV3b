/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
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
import frc.robot.interfaces.ISubsystem;
import frc.robot.models.LogDataBE;


public class Carriage extends Subsystem implements ISubsystem 
{
  // define class level working variables

    private TalonSRX leftSwitchblade;
    private TalonSRX rightSwitchblade;

    private boolean isLeftZeroed = false;
    private boolean isRightZeroed = false;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
    private static Carriage _instance = new Carriage();
	
	public static Carriage getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
    private Carriage() {
        leftSwitchblade = new TalonSRX(RobotMap.LEFT_SWITCHBLADE_MOTOR_CAN_ADDRESS);
        rightSwitchblade = new TalonSRX(RobotMap.RIGHT_SWITCHBLADE_MOTOR_CAN_ADDRESS);

        leftSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
        rightSwitchblade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

        leftSwitchblade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        rightSwitchblade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

        leftSwitchblade.config_kF(0, 0.3354098361, 0);
        leftSwitchblade.config_kP(0, 1.5, 0);
        leftSwitchblade.config_kI(0, 0, 0);
        leftSwitchblade.config_kD(0, 0, 0);
        leftSwitchblade.configMotionAcceleration(2000, 0);
        leftSwitchblade.configMotionCruiseVelocity(3000, 0);

        rightSwitchblade.config_kF(0, 0.3354098361, 0);
        rightSwitchblade.config_kP(0, 1.5, 0);
        rightSwitchblade.config_kI(0, 0, 0);
        rightSwitchblade.config_kD(0, 0, 0);
        rightSwitchblade.configMotionAcceleration(2000, 0);
        rightSwitchblade.configMotionCruiseVelocity(3000, 0);
    }

    public boolean leftLimitSwitchPressed() {
        return !leftSwitchblade.getSensorCollection().isRevLimitSwitchClosed();
    }

    public boolean rightLimitSwitchPressed() {
        return !rightSwitchblade.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void driveLeft(Double power) {
        leftSwitchblade.set(ControlMode.PercentOutput, power);
    }
    
    public void driveRight(Double power) {
        rightSwitchblade.set(ControlMode.PercentOutput, power);
    }

    public void driveBoth(Double power) {
        driveLeft(power);
        driveRight(power);
    }

    public void homeLeft() {
        if (leftLimitSwitchPressed()) {
            // Zero it and stop
            isLeftZeroed = true;
            leftSwitchblade.setSelectedSensorPosition(0, 0, 0);
            this.driveLeft(0.0);
             
        } else if (!isLeftZeroed) {
            // Drive
            this.driveLeft(-0.13);
        }
    }

    public void homeRight() {
        if (rightLimitSwitchPressed()) {
            // Zero it and stop
            isRightZeroed = true;      
            rightSwitchblade.setSelectedSensorPosition(0, 0, 0);
            this.driveRight(0.0);
        } else if (!isRightZeroed) {
            // Drive
            this.driveRight(0.13);
        }
    }

    public boolean getIsZeroed() {
        return isLeftZeroed && isRightZeroed;
    }

    public void resetZeroedStatus() {
        isLeftZeroed = false;
        isRightZeroed = false;
    }

    public void goToLeftPosition(Double degrees) {
        leftSwitchblade.set(ControlMode.MotionMagic, degreesToPulses(degrees));
    }

    public void goToRightPosition(Double degrees) {
        rightSwitchblade.set(ControlMode.MotionMagic, degreesToPulses(degrees));
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
        //SmartDashboard.putString("State: Carriage", get_carriageWheelsState().toString());
        SmartDashboard.putBoolean("Left Limit Switch", leftLimitSwitchPressed());
        SmartDashboard.putBoolean("Right Limit Switch", rightLimitSwitchPressed());

        SmartDashboard.putNumber("Left Motor Pulses", leftSwitchblade.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Right Motor Pulses", rightSwitchblade.getSelectedSensorPosition(0));

        SmartDashboard.putNumber("Left Motor Degrees", pulsesToDegrees(leftSwitchblade.getSelectedSensorPosition(0)));
        SmartDashboard.putNumber("Right Motor Degrees", pulsesToDegrees(rightSwitchblade.getSelectedSensorPosition(0)));

        SmartDashboard.putBoolean("Left Zeroed", isLeftZeroed);
        SmartDashboard.putBoolean("Right Zeroed", isRightZeroed);
    }

    public double pulsesToDegrees(int pulses) {
        return 360 * pulses / 4096;
    }

    public int degreesToPulses(double degrees) {
        Double pulses = (4096 * degrees / 360);
        return pulses.intValue();
    }
}
