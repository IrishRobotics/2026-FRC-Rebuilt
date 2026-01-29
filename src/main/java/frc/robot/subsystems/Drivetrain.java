// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private double speed = Constants.Drivetrain.lowSpeed;

  private MecanumDrive drive =
      new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  public Drivetrain() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    defaultConfig.inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);

    frontLeftMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    frontRightMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backLeftMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backRightMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void drive(double forward, double strafe, double turn) {
    drive.driveCartesian(forward * speed, strafe * speed, turn * speed);
  }

  public Command operatorDrive(CommandXboxController controller) {
    return new RunCommand(
        () -> {
          this.drive(controller.getRightY(), controller.getRightX(), controller.getLeftX());
        },
        this);
  }

  /*
   * Gets all motors for use by tests and sets the deadband to zero.
   *
   * @returns {fl, fr, bl, br}
   */
  public SparkMax[] testMode() {
    drive.setDeadband(0);
    SparkMax[] motors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
    return motors;
  }

  public void setSpeed(double newSpeed) throws IllegalArgumentException {
    if (newSpeed > 1 || newSpeed < 0)
      throw new IllegalArgumentException(
          String.format("Value %.1f not in range [0.0, 1.0]", newSpeed));

    speed = newSpeed;
  }

  public double getSpeed() {
    return speed;
  }

  public Command toggleSpeed() {
    return new InstantCommand(
        () -> {
          if (speed == Constants.Drivetrain.highSpeed) speed = Constants.Drivetrain.lowSpeed;
          else speed = Constants.Drivetrain.highSpeed;
        });
  }

  @Override
  public void close() {
    frontLeftMotor.close();
    frontRightMotor.close();
    backLeftMotor.close();
    backRightMotor.close();
  }
}
