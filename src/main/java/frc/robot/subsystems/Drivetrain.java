// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
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
  private Pigeon2 imu = new Pigeon2(Constants.Sensors.PIGEON_ID);
  private Translation2d frontLeftTranslate =
      new Translation2d(
          Constants.Drivetrain.WHEEL_LENGTH / 2, Constants.Drivetrain.WHEEL_WIDTH / 2);
  private Translation2d frontRightTranslate =
      new Translation2d(
          Constants.Drivetrain.WHEEL_LENGTH / 2, -Constants.Drivetrain.WHEEL_WIDTH / 2);
  private Translation2d backLeftTranslate =
      new Translation2d(
          -Constants.Drivetrain.WHEEL_LENGTH / 2, Constants.Drivetrain.WHEEL_WIDTH / 2);
  private Translation2d backRightTranslate =
      new Translation2d(
          -Constants.Drivetrain.WHEEL_LENGTH / 2, -Constants.Drivetrain.WHEEL_WIDTH / 2);
  private MecanumDriveKinematics kinematics =
      new MecanumDriveKinematics(
          frontLeftTranslate, frontRightTranslate, backLeftTranslate, backRightTranslate);
  private Pose2d robotPose = new Pose2d();
  private MecanumDriveOdometry odometry =
      new MecanumDriveOdometry(kinematics, imu.getRotation2d(), getPositions(), robotPose);

  private MecanumDrive drive =
      new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  /** Creates a new {@code Drivetrain} using the constants in {@code Constants} */
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

  @Override
  public void periodic() {
    robotPose = odometry.update(imu.getRotation2d(), getPositions());
  }

  /**
   * @return the robot's pose
   */
  public Pose2d getPose() {
    return robotPose;
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
   * @param squareInputs whether to square the controller inputs
   * @return a command that moves according to controller input
   */
  public Command operatorDrive(CommandXboxController controller, boolean squareInputs) {
    return new RunCommand(
        () -> {
          this.drive(-controller.getRightY(), controller.getRightX(), controller.getLeftX());
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
    if (imu != null) {
      imu.close();
    }
  }

  private MecanumDriveWheelPositions getPositions() {
    return new MecanumDriveWheelPositions(
        frontLeftMotor.getEncoder().getPosition(),
        frontRightMotor.getEncoder().getPosition(),
        backLeftMotor.getEncoder().getPosition(),
        backRightMotor.getEncoder().getPosition());
  }
}
