// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Debug;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/** The main robot controller class */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController coopController;
  private final UsbCamera shootCamera;

  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Shooter shooter;

  /** A controller for the robot */
  public RobotContainer() {
    driverController = new CommandXboxController(Constants.Control.DRIVER_CONTROLLER_PORT);
    coopController = new CommandXboxController(Constants.Control.COOP_CONTROLLER_PORT);

    shootCamera =  CameraServer.startAutomaticCapture();
    shootCamera.setResolution(480, 360);
   

    drivetrain = new Drivetrain();
    intake = new Intake();
    shooter = new Shooter();

    drivetrain.setDefaultCommand(drivetrain.operatorDrive(driverController));

    configureBindings();
  }

  private void configureBindings() {
    driverController.start().whileTrue(Debug.triggered("Driver A"));
    // coopController.start().whileTrue(Debug.triggered("Coop A"));

    coopController.rightTrigger().whileTrue(intake.WheelIn());
    coopController.leftTrigger().whileTrue(shooter.RunShooter());
    coopController.y().whileTrue(intake.ArmUp());
    coopController.a().whileTrue(intake.ArmDown());
    coopController.povUp().whileTrue(shooter.RunFeeder());
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
