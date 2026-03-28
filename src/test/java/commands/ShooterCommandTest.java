package commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import general.subsystems.ShooterTests;
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

    final double[] speeds = simulate(10);
    assertTrue(speeds[0] > 1);
    assertTrue(speeds[1] > 1);

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

    final double[] speeds = simulate(10);
    assertTrue(speeds[0] > 1);
    assertTrue(speeds[1] > 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
  }

  @Test
  void runShooter() {
    Command testCommand = shooter.runShooter();
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(Constants.Feeder.FEEDER_POWER, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    final double[] speeds = simulate(40);
    assertEquals(Constants.Shooter.SHOOTER_RPM, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(Constants.Shooter.SHOOTER_RPM, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    assertTrue(speeds[0] > 1);
    assertTrue(speeds[1] > 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
  }
}
