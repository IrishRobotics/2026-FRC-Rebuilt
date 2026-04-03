// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Retention;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DumbAuto;
import frc.robot.commands.LessDumbAuto;
import frc.robot.commands.MoveFront;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.*;

/** The main robot controller class */
public class RobotContainer {
  private final CommandXboxController driverController;
  private final CommandXboxController coopController;
  private final UsbCamera shootCamera;

  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Feeder feeder;
  private final Arm arm;
  private final Intake intake;

  private SendableChooser<Command> auto = new SendableChooser<>();

  /** A controller for the robot */
  public RobotContainer() {
    driverController = new CommandXboxController(Constants.Control.DRIVER_CONTROLLER_PORT);
    coopController = new CommandXboxController(Constants.Control.COOP_CONTROLLER_PORT);

    shootCamera = CameraServer.startAutomaticCapture();
    shootCamera.setResolution(480, 360);

    drivetrain = new Drivetrain();
    intake = new Intake();
    shooter = new Shooter();
    feeder = new Feeder();
    arm = new Arm();

    drivetrain.setDefaultCommand(drivetrain.operatorDrive(driverController, true));

    auto.setDefaultOption("Nothing", new PrintCommand("NO AUTO"));
    auto.addOption("Move n shoot", new LessDumbAuto(drivetrain, shooter, feeder));
    auto.addOption("Just shoot", new ShootCommand(shooter, feeder).withDeadline(new WaitCommand(8)));

    SmartDashboard.putData(auto);//9.999999696126e-8

    configureBindings();
  }

  private void configureBindings() {
    // driverController.a().onTrue(drivetrain.toggleSpeed());
    driverController
        .a()
        .onTrue(Commands.runOnce(() -> drivetrain.setSpeed(Constants.Drivetrain.HIGH_SPEED)));
    driverController
        .b()
        .onTrue(Commands.runOnce(() -> drivetrain.setSpeed(Constants.Drivetrain.LOW_SPEED)));

    coopController.rightTrigger().whileTrue(intake.runIntake(Constants.Intake.WHEEL_SPEED));
    coopController.leftTrigger().whileTrue(new ShootCommand(shooter, feeder));
    coopController.y().whileTrue(arm.runArm(0.5));
    coopController.a().whileTrue(arm.runArm(-0.5));
    coopController.povUp().whileTrue(feeder.run(-Constants.Feeder.FEEDER_POWER));
    coopController
        .povDown()
        .whileTrue(
            shooter.runAtSpeed(Constants.Shooter.SHOOTER_RPM, Constants.Shooter.SHOOTER_RPM));
    coopController.povLeft().whileTrue(intake.runIntake(-1));
    coopController.leftBumper().whileTrue(shooter.shootScale(0.1));
    coopController.rightBumper().whileTrue(shooter.shootScale(-0.1));

  }

  /**
   * Retrieves the selected autonomous command
   *
   * @return autonomous command
   */
  public Command getAutonomousCommand() {
    // return new LessDumbAuto(drivetrain, shooter, feeder);
    return auto.getSelected();
  }
}
