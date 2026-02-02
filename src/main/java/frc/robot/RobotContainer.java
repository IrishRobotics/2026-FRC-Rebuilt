// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Debug;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

/** The main robot controller class */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController coopController;

  private final Drivetrain drivetrain;
  private final Shooter shooter;

  /** A controller for the robot */
  public RobotContainer() {
    driverController = new CommandXboxController(Constants.Control.DRIVER_CONTROLLER_PORT);
    coopController = new CommandXboxController(Constants.Control.COOP_CONTROLLER_PORT);

    drivetrain = new Drivetrain();
    shooter = new Shooter();

    drivetrain.setDefaultCommand(drivetrain.operatorDrive(driverController));

    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(Debug.triggered("Driver A"));
    coopController.a().whileTrue(Debug.triggered("Coop A"));
    coopController.b().whileTrue(shooter.runAtSpeed(1000));
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
