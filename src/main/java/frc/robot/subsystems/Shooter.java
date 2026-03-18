// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/** This class controls the robot's shooter */
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkMax topMotor = new SparkMax(Constants.Shooter.TOP_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder topMotorEncoder = topMotor.getEncoder();
  private final SparkClosedLoopController topMotorController = topMotor.getClosedLoopController();
  private final SparkMax bottomMotor =
      new SparkMax(Constants.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder bottomMotorEncoder = bottomMotor.getEncoder();
  private final SparkClosedLoopController bottomMotorController =
      bottomMotor.getClosedLoopController();
  private final SparkMax feederMotor =
      new SparkMax(Constants.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

  /** Creates a new shooter with the values in Constants */
  public Shooter() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    // TODO: tune PID loop
    defaultConfig
        .closedLoop
        .p(Constants.Shooter.PID_P)
        .i(Constants.Shooter.PID_I)
        .d(Constants.Shooter.PID_D)
        .outputRange(0, 1);
    defaultConfig.encoder.velocityConversionFactor(1);
    defaultConfig.idleMode(IdleMode.kCoast);
    defaultConfig.smartCurrentLimit(30);

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
    if (!Robot.isSimulation()) {
      SmartDashboard.putNumber("Shooter: Top Motor", topMotorEncoder.getVelocity());
      SmartDashboard.putNumber("Shooter: Bottom Motor", bottomMotorEncoder.getVelocity());
    }
  }

  /** Stops both motors */
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
    feederMotor.stopMotor();
  }

  /**
   * Sets the speed of the top and bottom motors separately
   *
   * @param topSpeed top setpoint in RPM
   * @param bottomSpeed bottom setpoint in RPM
   */
  public void setSpeed(double topSpeed, double bottomSpeed) {
    topMotorController.setSetpoint(topSpeed, ControlType.kVelocity);
    bottomMotorController.setSetpoint(bottomSpeed, ControlType.kVelocity);
  }

  /**
   * Sets the speed of both the top and bottom motor
   *
   * @param speed setpoint in RPM
   */
  public void setSpeed(double speed) {
    setSpeed(speed, speed);
  }

  /**
   * Sets the speed of the feeder motor
   *
   * @param speed
   */
  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Creates a command that runs the shooter at the given speeds
   *
   * @param speed The speed for both motors (RPM)
   * @return A command that runs the shooter at the given speeds
   */
  public Command runAtSpeed(double speed) {
    return runAtSpeed(speed, speed);
  }

  /**
   * Creates a command that runs the shooter at the given speeds
   *
   * @param topSpeed The speed for the top motor (RPM)
   * @param bottomSpeed The speed for the bottom motor (RPM)
   * @return A command that runs the shooter at the given speeds
   */
  public Command runAtSpeed(double topSpeed, double bottomSpeed) {
    return this.startEnd(() -> setSpeed(topSpeed, bottomSpeed), () -> stop());
  }

  public Command runFeeder(double speed) {
    return this.startEnd(
        () -> {
          feederMotor.set(speed);
        },
        () -> {
          feederMotor.stopMotor();
        });
  }

  public Command runShooter() {
    return this.startEnd(
        () -> {
          setSpeed(Constants.Shooter.SHOOTER_RPM);
          Timer.delay(Constants.Shooter.FEEDER_WAIT);
          setFeederSpeed(Constants.Shooter.FEEDER_POWER);
        },
        () -> {
          setSpeed(0);
          setFeederSpeed(0);
        });
  }

  @Override
  public void close() {
    topMotor.close();
    bottomMotor.close();
    feederMotor.close();
  }
}
