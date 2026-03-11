// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonSRX pivotMotor = new TalonSRX(Constants.Arm.ARM_MOTOR);

  /** Creates a new Arm. */
  public Arm() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.PulseWidthEncodedPosition;
    //TODO: Set up encoder
    config.slot0.kP = Constants.Arm.PID_P;
    config.slot0.kI = Constants.Arm.PID_I;
    config.slot0.kD = Constants.Arm.PID_D;
    pivotMotor.configAllSettings(config);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void stop() {
    setArmSpeed(0);
  }

  public void setArmSpeed(double speed) {
    pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setArmTarget(double target) {
    pivotMotor.set(TalonSRXControlMode.Position,target);
  }

  public Command setArm(double target) {
    return this.runOnce(() -> setArmTarget(target));
  }

  public Command runArm(double speed) {
    return this.startEnd(() -> setArmSpeed(speed), this::stop);
  }
}
