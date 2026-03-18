// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceCommand extends Command {
  private final Drivetrain drivetrain;
  private Translation2d offsetPose;
  private final Pose2d targetPose;
  private final boolean relative;
  private final double speed;
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  /** Creates a new DriveDistanceCommand. */
  public DriveDistanceCommand(Drivetrain drivetrain, Pose2d targetPose, boolean relative, double speed) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    this.relative = relative;
    this.speed = speed;

    addRequirements(drivetrain);
  }

  public DriveDistanceCommand(Drivetrain drivetrain, double targetX, double targetY, double targetRotation, boolean relative, double speed) {
    this(drivetrain, new Pose2d(targetX, targetY, Rotation2d.fromRotations(targetRotation)), relative, speed);
  }

  @Override
  public void initialize() {
    if(relative) {
      offsetPose = targetPose.getTranslation().plus(drivetrain.getPose().getTranslation());
    } else {
      offsetPose = targetPose.getTranslation();
    }
  }

  @Override
  public void execute() {
    Translation2d difference = offsetPose.minus(drivetrain.getPose().getTranslation());
    drivetrain.drive(
        xSpeedLimiter.calculate(speed * Math.max(-1, Math.min(1,difference.getX()))),
        ySpeedLimiter.calculate(speed * Math.max(-1, Math.min(1,difference.getY()))),
        rotLimiter.calculate(targetPose.getRotation().getRotations()));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
