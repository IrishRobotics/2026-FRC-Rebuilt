// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arm extends SubsystemBase implements AutoCloseable {
  private TalonSRX pivotMotor = new TalonSRX(Constants.Arm.ARM_MOTOR);
  private PIDController pidController =
      new PIDController(Constants.Arm.PID_P, Constants.Arm.PID_I, Constants.Arm.PID_D);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);
  private boolean isPIDEnabled = false;

  /** Creates a new Arm. */
  public Arm() {
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    pivotMotor.configAllSettings(config);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    if (isPIDEnabled) {
      double pidOutput = pidController.calculate(encoder.get());
      pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
      pivotMotor.set(TalonSRXControlMode.PercentOutput, pidOutput);
    }
    if (!Robot.isSimulation()) {
      SmartDashboard.putNumber("Arm Encoder", encoder.get());
      SmartDashboard.putNumber("Arm Setpoint", pidController.getSetpoint());
      SmartDashboard.putNumber("Arm Output", pivotMotor.getMotorOutputPercent());
    }
  }

  public void stop() {
    setArmSpeed(0);
  }

  public void setArmSpeed(double speed) {
    isPIDEnabled = false;
    pivotMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void setArmTarget(double target) {
    pidController.setSetpoint(target);
    isPIDEnabled = true;
  }

  public Command setArm(double target) {
    return this.runOnce(() -> setArmTarget(target));
  }

  public Command runArm(double speed) {
    return this.startEnd(() -> setArmSpeed(speed), this::stop);
  }

  public void close() {
    encoder.close();
  }
}
