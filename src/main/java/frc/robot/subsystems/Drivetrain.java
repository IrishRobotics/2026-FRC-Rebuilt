// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private SparkMax frontLeftMotor = 
    new SparkMax(Constants.Drivetrain.frontLeft, MotorType.kBrushless);
  private SparkMax frontRightMotor =
    new SparkMax(Constants.Drivetrain.frontRight, MotorType.kBrushless);
  private SparkMax backLeftMotor =
    new SparkMax(Constants.Drivetrain.backLeft, MotorType.kBrushless);
  private SparkMax backRightMotor = 
    new SparkMax(Constants.Drivetrain.backRight, MotorType.kBrushless);

  private MecanumDrive drive =
    new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

  public Drivetrain() {}

  public void Drive(double x, double y, double turn) {
    drive.driveCartesian(x, y, turn);
  }

  // public OperatorDrive(XboxController controller) {
    
  // }

  // // public Command OperatorDrive(XboxController controller) {
  // //   return new FunctionalCommand(
  // //     () -> {}, 
  // //     () -> {this.Drive(
  // //       controller.getRightX(),
  // //       controller.getLeftY(), 
  // //       controller.getLeftX());},
  // //     (v) -> {}, () -> {return false;},
  // //     this
  // //   );
  // // }
}
