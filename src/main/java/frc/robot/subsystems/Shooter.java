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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private double shootSpeed = Constants.Shooter.SHOOTER_RPM;

  /** Creates a new shooter with the values in Constants */
  public Shooter() {
    SparkMaxConfig defaultConfig = new SparkMaxConfig();
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
    return this.startEnd(() -> setSpeed(topSpeed, bottomSpeed), this::stop);
  }

  private class RunShooterCommand extends Command {
    @Override
    public void initialize() {
      runAtSpeed(shootSpeed);
    }

    @Override
    public void execute() {
      runAtSpeed(shootSpeed);
    }

    @Override
    public void end(boolean interrupted) {
      stop();
    }
  }

  public Command runShooter() {
    return new RunShooterCommand();
//    return this.startEnd(() -> setSpeed(Constants.Shooter.SHOOTER_RPM), this::stop);
  }

  public Command shootScale(double factor) {
    return Commands.startEnd(() -> shootSpeed = Constants.Shooter.SHOOTER_RPM * (1+factor), () -> shootSpeed = Constants.Shooter.SHOOTER_RPM);
  }

  @Override
  public void close() {
    topMotor.close();
    bottomMotor.close();
  }
}
