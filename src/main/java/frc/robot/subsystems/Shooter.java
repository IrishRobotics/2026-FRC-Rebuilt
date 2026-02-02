// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** The class for the Shooter subsystem */
public class Shooter extends SubsystemBase implements AutoCloseable {
  private final SparkMax topMotor = new SparkMax(Constants.Shooter.TOP_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController topMotorController = topMotor.getClosedLoopController();
  private final SparkMax bottomMotor = new SparkMax(Constants.Shooter.BOTTOM_MOTOR, MotorType.kBrushless);
  private final SparkClosedLoopController bottomMotorController = bottomMotor.getClosedLoopController();

  public Shooter() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
    // TODO: tune PID loop
    defaultConfig.closedLoop.p(Constants.Shooter.PID_P).d(Constants.Shooter.PID_D).i(Constants.Shooter.PID_I)
        .outputRange(0, 1);
    defaultConfig.encoder.velocityConversionFactor(1);
    SparkMaxConfig invertedConfig = new SparkMaxConfig().apply(defaultConfig);
    invertedConfig.inverted(true);
    // TODO: find what motor needs to be inverted
    bottomMotor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    topMotor.configure(invertedConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Sets the speed of the top and bottom variable separately
   * @param topSpeed top setpoint in RPM
   * @param bottomSpeed bottom setpoint in RPM
   */
  public void setSpeed(double topSpeed, double bottomSpeed) {
    topMotorController.setSetpoint(topSpeed, ControlType.kVelocity);
    bottomMotorController.setSetpoint(bottomSpeed, ControlType.kVelocity);
  }

  /** Sets the speed of both the top and bottom motor
   * @param speed setpoint in RPM
   */
  public void setSpeed(double speed) {
    setSpeed(speed, speed);
  }

  @Override
  public void close() {
    topMotor.close();
    bottomMotor.close();  
  }
}
