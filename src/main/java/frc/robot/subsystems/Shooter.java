// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private SparkMax topMotor = new SparkMax(Constants.Shooter.TOP_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController topMotorController = topMotor.getClosedLoopController();
  private SparkMax bottomMotor = new SparkMax(Constants.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController bottomMotorController = bottomMotor.getClosedLoopController();
  private SparkMax feederMotor = new SparkMax(Constants.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

  /** Creates a new shooter. */
  public Shooter() {

    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    defaultConfig.inverted(false);
    defaultConfig.idleMode(IdleMode.kCoast);
    defaultConfig.closedLoop.p(Constants.Shooter.PID_P).i(Constants.Shooter.PID_I).d(Constants.Shooter.PID_D);
    defaultConfig.encoder.velocityConversionFactor(1);

    SparkMaxConfig invertedConfig = new SparkMaxConfig().apply(defaultConfig);
    invertedConfig.inverted(true);

    topMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double topSpeed, double bottomSpeed) {
    topMotorController.setSetpoint(topSpeed, ControlType.kVelocity);
    bottomMotorController.setSetpoint(bottomSpeed, ControlType.kVelocity);
  }

  public Command RunFeeder(){
    return new StartEndCommand(
        () -> {
          feederMotor.set(0.5);
        },
        () -> {
          feederMotor.stopMotor();
        },
        this);}

  public Command RunShooter() {
    return new StartEndCommand(
        () -> {
          setSpeed(Constants.Shooter.WHEEL_SPEED, Constants.Shooter.WHEEL_SPEED);
          Timer.delay(0.5);
          feederMotor.set(0.5);
        },
        () -> {
          topMotor.stopMotor();
          bottomMotor.stopMotor();
          feederMotor.stopMotor();
        },
        this);
  }
}
