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

/** This class controls the robot's Mecanum drivebase */
public class Drivetrain extends SubsystemBase implements AutoCloseable {
  private SparkMax frontLeftMotor =
      new SparkMax(Constants.Drivetrain.FRONT_LEFT_MOTOR, MotorType.kBrushless);
  private SparkMax frontRightMotor =
      new SparkMax(Constants.Drivetrain.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
  private SparkMax backLeftMotor =
      new SparkMax(Constants.Drivetrain.BACK_LEFT_MOTOR, MotorType.kBrushless);
  private SparkMax backRightMotor =
      new SparkMax(Constants.Drivetrain.BACK_RIGHT_MOTOR, MotorType.kBrushless);
  private double speed = Constants.Drivetrain.LOW_SPEED;

  private MecanumDrive drive =
      new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  /** Creates a new {@code Drivetrain} using the constants in {@code Constants} */
  public Drivetrain() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    defaultConfig.inverted(false);
    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);

    frontLeftMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontRightMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the motor speeds using a robot centric drive style
   *
   * @param forward the component along the forward axis of the robot
   * @param strafe the component perpendicular to the forward component
   * @param turn the turn component of the movement
   */
  public void drive(double forward, double strafe, double turn) {
    drive.driveCartesian(forward * speed, strafe * speed, turn * speed);
  }

  /**
   * A command factory for driving controlled by a controller
   *
   * @param controller the controller movement will be pulled from
   * @return a command that moves according to controller input
   */
  public Command operatorDrive(CommandXboxController controller) {
    return new RunCommand(
        () -> {
          this.drive(-controller.getLeftY(), controller.getLeftX(), controller.getRightX());
        },
        this);
  }

  /**
   * Sets the max speed of the robot
   *
   * @param newSpeed [0,1]
   */
  public void setSpeed(double newSpeed) throws IllegalArgumentException {
    if (newSpeed > 1 || newSpeed < 0)
      throw new IllegalArgumentException(
          String.format("Value %.1f not in range [0.0, 1.0]", newSpeed));

    speed = newSpeed;
  }

  /**
   * Returns the current robot speed
   *
   * @return current speed
   */
  public double getSpeed() {
    return speed;
  }

  /**
   * Creates an instantaneous command to switch between high and low speed
   *
   * @return a command to toggle the speed
   */
  public Command toggleSpeed() {
    return new InstantCommand(
        () -> {
          if (speed == Constants.Drivetrain.HIGH_SPEED) setSpeed(Constants.Drivetrain.LOW_SPEED);
          else setSpeed(Constants.Drivetrain.HIGH_SPEED);
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
