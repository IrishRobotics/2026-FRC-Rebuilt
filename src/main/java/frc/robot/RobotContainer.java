// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Debug;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController coopController;

  private final Drivetrain drivetrain;

  public RobotContainer() {
    driverController = new CommandXboxController(Constants.Control.driverControllerPort);
    coopController = new CommandXboxController(Constants.Control.coopControllerPort);

    drivetrain = new Drivetrain();

    drivetrain.setDefaultCommand(drivetrain.operatorDrive(driverController));

    configureBindings();
  }

  private void configureBindings() {
    driverController.a().whileTrue(Debug.triggered("Driver A"));
    coopController.a().whileTrue(Debug.triggered("Coop A"));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
