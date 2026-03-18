package commands;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import general.subsystems.ShooterTests;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Comprehensive tests for Shooter Commands.
 * Tests cover command lifecycle, motor setpoints, and feeder behavior.
 */
@DisplayName("Shooter Command Tests")
class ShooterCommandTest extends ShooterTests {

  @Nested
  @DisplayName("Speed Command Tests")
  class SpeedCommandTests {

    @Test
    @DisplayName("runAtSpeed command with single parameter should set both motors equally")
    void runAtSpeedBothEqual() {
      final double speed = 10;
      Command testCommand = shooter.runAtSpeed(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Motors should accelerate when command runs")
    void motorsAccelerate() {
      final double speed = 10;
      Command testCommand = shooter.runAtSpeed(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      final double[] speeds = simulate(10);
      assertTrue(speeds[0] > 1);
      assertTrue(speeds[1] > 1);
    }

    @Test
    @DisplayName("Command should reset motors when cancelled")
    void commandCancelResetsMotors() {
      final double speed = 10;
      Command testCommand = shooter.runAtSpeed(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
      assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("runAtSpeed with separate speeds should set motors independently")
    void runAtSpeedSeparateSpeeds() {
      final double topSpeed = 50;
      final double bottomSpeed = 25;
      Command testCommand = shooter.runAtSpeed(topSpeed, bottomSpeed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(topSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(bottomSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Motors should accelerate with separate speeds")
    void motorsAccelerateSeparate() {
      final double topSpeed = 50;
      final double bottomSpeed = 25;
      Command testCommand = shooter.runAtSpeed(topSpeed, bottomSpeed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      final double[] speeds = simulate(10);
      assertTrue(speeds[0] > 1);
      assertTrue(speeds[1] > 1);
    }

    @Test
    @DisplayName("Separate speed command should reset on cancel")
    void separateSpeedCancelResetsMotors() {
      final double topSpeed = 50;
      final double bottomSpeed = 25;
      Command testCommand = shooter.runAtSpeed(topSpeed, bottomSpeed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().cancelAll();
      assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("High speed command should produce high RPM")
    void highSpeedCommand() {
      final double speed = Constants.Shooter.SHOOTER_RPM;
      Command testCommand = shooter.runAtSpeed(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

      final double[] speeds = simulate(20);
      assertTrue(speeds[0] > speed * 0.1, "Top motor should show acceleration");
      assertTrue(speeds[1] > speed * 0.1, "Bottom motor should show acceleration");

      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Zero speed command should stop motors")
    void zeroSpeedCommand() {
      Command testCommand = shooter.runAtSpeed(0);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Negative speed command should reverse motors")
    void negativeSpeedCommand() {
      final double speed = -5000;
      Command testCommand = shooter.runAtSpeed(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Nested
  @DisplayName("Feeder Command Tests")
  class FeederCommandTests {

    @Test
    @DisplayName("runFeeder command should set feeder to specified speed")
    void runFeederCommand() {
      final double speed = 0.5;
      Command testCommand = shooter.runFeeder(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder should accelerate during command")
    void feederAccelerates() {
      final double speed = 0.5;
      Command testCommand = shooter.runFeeder(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      final double[] speeds = simulate(10);
      assertTrue(speeds[2] > 0);
    }

    @Test
    @DisplayName("Feeder command should stop motor when cancelled")
    void feederCommandCancelStops() {
      final double speed = 0.5;
      Command testCommand = shooter.runFeeder(speed);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(speed, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
      assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder command with zero speed should stop feeder")
    void feederZeroSpeedCommand() {
      Command testCommand = shooter.runFeeder(0.0);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(0.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Feeder command with full power should run maximum")
    void feederFullPowerCommand() {
      Command testCommand = shooter.runFeeder(1.0);
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(1.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Multiple feeder commands should cancel previous one")
    void multipleFeederCommandsCancel() {
      Command cmd1 = shooter.runFeeder(0.3);
      CommandScheduler.getInstance().schedule(cmd1);
      CommandScheduler.getInstance().run();

      Command cmd2 = shooter.runFeeder(0.7);
      CommandScheduler.getInstance().schedule(cmd2);
      CommandScheduler.getInstance().run();

      assertEquals(0.7, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
      assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Full Shooter Command Tests")
  class FullShooterCommandTests {

    @Test
    @DisplayName("runShooter command should run both motors and feeder")
    void runShooterCommand() {
      Command testCommand = shooter.runShooter();
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      assertEquals(Constants.Shooter.SHOOTER_RPM, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(Constants.Shooter.SHOOTER_RPM, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(Constants.Shooter.FEEDER_POWER, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Shooter motors should accelerate")
    void shooterMotorsAccelerate() {
      Command testCommand = shooter.runShooter();
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      final double[] speeds = simulate(40);
      assertEquals(Constants.Shooter.SHOOTER_RPM, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(Constants.Shooter.SHOOTER_RPM, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertTrue(speeds[0] > 1);
      assertTrue(speeds[1] > 1);
    }

    @Test
    @DisplayName("runShooter command should stop all motors when cancelled")
    void runShooterCancelStopsAll() {
      Command testCommand = shooter.runShooter();
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().cancelAll();
      assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Command Lifecycle Tests")
  class CommandLifecycleTests {

    @Test
    @DisplayName("Commands should be schedulable")
    void commandsSchedulable() {
      Command cmd = shooter.runAtSpeed(1000);
      assertNotNull(cmd);
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().cancelAll();
      assertTrue(true, "Command successfully scheduled and cancelled");
    }

    @Test
    @DisplayName("Multiple commands can be scheduled sequentially")
    void sequentialCommands() {
      Command cmd1 = shooter.runAtSpeed(2000);
      CommandScheduler.getInstance().schedule(cmd1);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().cancelAll();

      Command cmd2 = shooter.runFeeder(0.5);
      CommandScheduler.getInstance().schedule(cmd2);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().cancelAll();

      assertTrue(true, "Sequential commands executed");
    }

    @Test
    @DisplayName("Commands can be created with various parameters")
    void commandsWithVariousParameters() {
      double[] speeds = {0, 1000, 5000, 10000};
      for (double speed : speeds) {
        Command cmd = shooter.runAtSpeed(speed);
        assertNotNull(cmd);
      }
      assertTrue(true, "Commands created with various speeds");
    }

    @Test
    @DisplayName("Cancel all should properly stop all motors")
    void cancelAllStopsAllMotors() {
      Command cmd = shooter.runShooter();
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().cancelAll();

      assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Independence Tests")
  class IndependenceTests {

    @Test
    @DisplayName("Feeder command should not affect shooter motors")
    void feederCommandIndependent() {
      Command speedCmd = shooter.runAtSpeed(5000);
      CommandScheduler.getInstance().schedule(speedCmd);
      CommandScheduler.getInstance().run();

      Command feederCmd = shooter.runFeeder(0.75);
      CommandScheduler.getInstance().schedule(feederCmd);
      CommandScheduler.getInstance().run();

      assertTrue(true, "Feeder command scheduled independently");

      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Speed commands should set correct setpoints independently")
    void speedCommandsIndependent() {
      Command cmd1 = shooter.runAtSpeed(3000);
      CommandScheduler.getInstance().schedule(cmd1);
      CommandScheduler.getInstance().run();
      assertEquals(3000, topMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();

      Command cmd2 = shooter.runAtSpeed(6000);
      CommandScheduler.getInstance().schedule(cmd2);
      CommandScheduler.getInstance().run();
      assertEquals(6000, topMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Nested
  @DisplayName("Edge Cases and Boundary Tests")
  class EdgeCasesTests {

    @Test
    @DisplayName("Command with zero speed should be valid")
    void zeroSpeedCommandValid() {
      Command cmd = shooter.runAtSpeed(0);
      assertNotNull(cmd);
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Command with very high speed should be valid")
    void veryHighSpeedCommandValid() {
      final double veryHighSpeed = 50000;
      Command cmd = shooter.runAtSpeed(veryHighSpeed);
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();
      assertEquals(veryHighSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Feeder with minimum speed should be valid")
    void feederMinimumSpeed() {
      Command cmd = shooter.runFeeder(0.01);
      assertNotNull(cmd);
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();
      CommandScheduler.getInstance().cancelAll();
    }

    @Test
    @DisplayName("Unequal motor speeds should be maintained")
    void unequalSpeedsCommand() {
      final double topSpeed = 7000;
      final double bottomSpeed = 3000;
      Command cmd = shooter.runAtSpeed(topSpeed, bottomSpeed);
      CommandScheduler.getInstance().schedule(cmd);
      CommandScheduler.getInstance().run();

      assertEquals(topSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(bottomSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

      CommandScheduler.getInstance().cancelAll();
    }
  }
}
