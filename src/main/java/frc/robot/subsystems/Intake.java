// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX pivotMotor = new TalonSRX(Constants.Intake.PIVOT_MOTOR);
  private TalonSRX wheelMotor = new TalonSRX(Constants.Intake.WHEEL_MOTOR);

  /** Creates a new Intake. */
  public Intake() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command ArmUp() {
    return new StartEndCommand(this::ArmUpManual, this::StopArm, this);
  }

  public Command ArmDown() {
    return new StartEndCommand(this::ArmDownManual, this::StopArm, this);
  }

  public Command WheelIn() {
    return new StartEndCommand(this::In, this::StopWheel, this);
  }

  private void StopArm() {
    pivotMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  private void ArmUpManual() {
    pivotMotor.set(TalonSRXControlMode.PercentOutput, 0.5);
  }

  private void ArmDownManual() {
    pivotMotor.set(TalonSRXControlMode.PercentOutput, -0.5);
  }

  private void StopWheel() {
    wheelMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  private void In() {
    wheelMotor.set(TalonSRXControlMode.PercentOutput, Constants.Intake.WHEEL_SPEED);
  }

  private void Out() {
    wheelMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Intake.WHEEL_SPEED);
  }
}
