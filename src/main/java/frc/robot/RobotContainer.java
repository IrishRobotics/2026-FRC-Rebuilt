// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** The main robot controller class */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController coopController;
  private final UsbCamera shootCamera;

  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Arm arm;
  private final Intake intake;

  /** A controller for the robot */
  public RobotContainer() {
    driverController = new CommandXboxController(Constants.Control.DRIVER_CONTROLLER_PORT);
    coopController = new CommandXboxController(Constants.Control.COOP_CONTROLLER_PORT);

    shootCamera = CameraServer.startAutomaticCapture();
    shootCamera.setResolution(480, 360);

    drivetrain = new Drivetrain();
    intake = new Intake();
    shooter = new Shooter();
    arm = new Arm();

    drivetrain.setDefaultCommand(drivetrain.operatorDrive(driverController, true));

    configureBindings();
  }

  private void configureBindings() {
    coopController.rightTrigger().whileTrue(intake.runIntake(Constants.Intake.WHEEL_SPEED));
    coopController.leftTrigger().whileTrue(shooter.runShooter());
    coopController.y().whileTrue(arm.runArm(0.5));
    coopController.a().whileTrue(arm.runArm(-0.5));
    coopController.povUp().whileTrue(shooter.runFeeder(Constants.Shooter.FEEDER_POWER));
    coopController
        .povDown()
        .whileTrue(
            shooter.runAtSpeed(Constants.Shooter.SHOOTER_RPM, Constants.Shooter.SHOOTER_RPM));
  }

  /**
   * Retrieves the selected autonomous command
   *
   * @return autonomous command
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
