package commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import general.ShooterTests;

import org.junit.jupiter.api.Test;

class ShooterCommandTest extends ShooterTests {
  @Test
  void setBothSetpoints() {
    final double speed = 10;
    Command testCommand = shooter.runAtSpeed(speed);
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final AngularVelocity[] speeds = simulate(10);
    assertEquals(speed, speeds[0].abs(Units.RPM), 1);
    assertEquals(speed, speeds[1].abs(Units.RPM), 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
  }

  @Test
  void setBothSetpointsSeparate() {
    final double topSpeed = 50;
    final double bottomSpeed = 25;
    Command testCommand = shooter.runAtSpeed(topSpeed, bottomSpeed);
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(topSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(bottomSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final AngularVelocity[] speeds = simulate(10);
    assertEquals(topSpeed, speeds[0].abs(Units.RPM), 1);
    assertEquals(bottomSpeed, speeds[1].abs(Units.RPM), 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
  }

  @Test
  void setFeederSetpoint() {
    final double speed = 0.5;
    Command testCommand = shooter.runFeeder(speed);
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(feederMotor.getSetpoint(), speed);
    final AngularVelocity[] speeds = simulate(10);
    assertTrue(speeds[2].abs(Units.RPM) > 0);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
  }

  @Test
  void runShooter() {
    Command testCommand = shooter.runShooter();
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(Constants.Shooter.FEEDER_POWER, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    final AngularVelocity[] speeds = simulate(40);
    assertEquals(Constants.Shooter.SHOOTER_RPM, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(Constants.Shooter.SHOOTER_RPM, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(Constants.Shooter.SHOOTER_RPM, speeds[0].abs(Units.RPM), 1);
    assertEquals(Constants.Shooter.SHOOTER_RPM, speeds[1].abs(Units.RPM), 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
  }
}