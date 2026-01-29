// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase implements AutoCloseable {
  private SparkMax frontLeftMotor =
      new SparkMax(Constants.Drivetrain.frontLeft, MotorType.kBrushless);
  private SparkMax frontRightMotor =
      new SparkMax(Constants.Drivetrain.frontRight, MotorType.kBrushless);
  private SparkMax backLeftMotor =
      new SparkMax(Constants.Drivetrain.backLeft, MotorType.kBrushless);
  private SparkMax backRightMotor =
      new SparkMax(Constants.Drivetrain.backRight, MotorType.kBrushless);

  private MecanumDrive drive =
      new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  public Drivetrain() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    defaultConfig.inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);

    frontLeftMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backLeftMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    frontRightMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backRightMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void drive(double x, double y, double turn) {
    drive.driveCartesian(x, y, turn, new Rotation2d());
  }

  public Command operatorDrive(CommandXboxController controller) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          this.drive(controller.getRightX(), controller.getRightY(), controller.getLeftX());
        },
        (v) -> {},
        () -> {
          return false;
        },
        this);
  }

  public SparkMax[] getMotors() {
    SparkMax[] motors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
    return motors;
  }

  @Override
  public void close() {
    frontLeftMotor.close();
    frontRightMotor.close();
    backLeftMotor.close();
    backRightMotor.close();
  }
}
