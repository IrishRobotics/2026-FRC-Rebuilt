// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.Intake.WHEEL_MOTOR);

  /** Creates a new Intake. */
  public Intake() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    intakeMotor.configAllSettings(config);
    intakeMotor.setInverted(InvertType.InvertMotorOutput);
  }

  public void stop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void setWheelPercent(double percent) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, percent);
  }

  public Command runIntake(double percent) {
    return this.startEnd(() -> setWheelPercent(percent), this::stop);
  }
}
