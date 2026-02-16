// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private SparkMax topMotor = new SparkMax(Constants.Shooter.TOP_MOTOR, MotorType.kBrushless);
  private SparkMax bottomMotor = new SparkMax(Constants.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
  private SparkMax feederMotor = new SparkMax(Constants.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

  /** Creates a new shooter. */
  public Shooter() {

    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    defaultConfig.inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);

    topMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    bottomMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command RunShooter() {
    return new StartEndCommand(
        () -> {
          topMotor.set(Constants.Shooter.WHEEL_SPEED);
          bottomMotor.set(Constants.Shooter.WHEEL_SPEED);
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
