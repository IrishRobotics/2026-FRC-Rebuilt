// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** This class controls the robot's shooter */
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkMax topMotor = new SparkMax(Constants.Shooter.TOP_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController topMotorController = topMotor.getClosedLoopController();
  private final SparkMax bottomMotor =
      new SparkMax(Constants.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController bottomMotorController =
      bottomMotor.getClosedLoopController();

  /** Creates a new shooter with the values in Constants */
  public Shooter() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    // TODO: tune PID loop
    defaultConfig
        .closedLoop
        .p(Constants.Shooter.PID_P)
        .d(Constants.Shooter.PID_D)
        .i(Constants.Shooter.PID_I)
        .outputRange(0, 1);
    defaultConfig.encoder.velocityConversionFactor(1);
    defaultConfig.idleMode(IdleMode.kCoast);

    SparkMaxConfig invertedConfig = new SparkMaxConfig().apply(defaultConfig);
    invertedConfig.inverted(true);
    // TODO: find what motor needs to be inverted
    bottomMotor.configure(
        defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    topMotor.configure(
        invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Stops both motors */
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
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
   * Creates a command that runs the shooter at the given speeds
   *
   * @param speed The speed for both motors (RPM)
   * @return A command that runs the shooter at the given speeds
   */
  public Command runAtSpeed(double speed) {
    return new StartEndCommand(() -> setSpeed(speed), () -> stop(), this);
  }

  /**
   * Creates a command that runs the shooter at the given speeds
   *
   * @param topSpeed The speed for the top motor (RPM)
   * @param bottomSpeed The speed for the bottom motor (RPM)
   * @return A command that runs the shooter at the given speeds
   */
  public Command runAtSpeed(double topSpeed, double bottomSpeed) {
    return new StartEndCommand(() -> setSpeed(topSpeed, bottomSpeed), () -> stop(), this);
  }

  @Override
  public void close() {
    topMotor.close();
    bottomMotor.close();
  }
}
