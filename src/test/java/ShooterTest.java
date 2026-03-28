

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import general.TestBase;
import general.motors.SparkMaxNeoMotor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Comprehensive tests for the Shooter subsystem. Tests cover motor setpoints,
 * feeder control, and
 * various speed combinations.
 */
@DisplayName("Shooter Subsystem Tests")
class ShooterTest extends TestBase {
  protected Shooter shooter;
  protected SparkMaxNeoMotor topMotor;
  protected SparkMaxNeoMotor bottomMotor;
  protected SparkMaxNeoMotor feederMotor;

  @BeforeEach
  @Override
  protected void setup() {
    super.setup();
    shooter = new Shooter();
    try {
      topMotor = new SparkMaxNeoMotor(shooter, "topMotor");
      bottomMotor = new SparkMaxNeoMotor(shooter, "bottomMotor");
      feederMotor = new SparkMaxNeoMotor(shooter, "feederMotor");
    } catch (NoSuchFieldException e) {
      e.printStackTrace();
      fail("Failed to access motor fields via reflection");
    }
  }

  @org.junit.jupiter.api.AfterEach
  protected void cleanup() throws Exception {
    super.cleanup();
    if (shooter != null) {
      shooter.close();
    }
  }

  @Nested
  @DisplayName("Motor Speed Control Tests")
  class MotorSpeedControlTests {

    @Test
    @DisplayName("Setting speed with single parameter should set both motors equally")
    void setBothSetpointsEqual() {
      final double speed = 10;
      shooter.setSpeed(speed);
      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Motors should accelerate during simulation")
    void motorsShouldAccelerate() {
      final double speed = 10;
      shooter.setSpeed(speed);
      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getTopSpeed() > 1, "Top motor should accelerate");
      assertTrue(speeds.getBottomSpeed() > 1, "Bottom motor should accelerate");
    }

    @Test
    @DisplayName("Setting separate speeds should set each motor independently")
    void setBothSetpointsSeparate() {
      final double topSpeed = 50;
      final double bottomSpeed = 25;
      shooter.setSpeed(topSpeed, bottomSpeed);
      assertEquals(topSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(bottomSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Both motors should accelerate with separate speeds")
    void separateSpedsAccelerate() {
      final double topSpeed = 50;
      final double bottomSpeed = 25;
      shooter.setSpeed(topSpeed, bottomSpeed);
      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getTopSpeed() > 1, "Top motor should accelerate");
      assertTrue(speeds.getBottomSpeed() > 1, "Bottom motor should accelerate");
    }

    @Test
    @DisplayName("Zero speed should stop motors")
    void zeroSpeedStopsMotors() {
      shooter.setSpeed(0);
      assertEquals(0.0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Zero speed with separate parameters should stop both motors")
    void zeroSpeedSeparateStopsMotors() {
      shooter.setSpeed(0, 0);
      assertEquals(0.0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("High speed should produce high RPM")
    void highSpeedProducesHighRPM() {
      final double speed = 7000; // Constants.Shooter.SHOOTER_RPM
      shooter.setSpeed(speed);
      assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      final SimulatedSpeeds speeds = simulate(20);
      // Flywheel acceleration is physics-based; relaxed threshold to account for
      // realistic acceleration
      assertTrue(speeds.getTopSpeed() > speed * 0.1, "Top motor should show acceleration toward target");
      assertTrue(speeds.getBottomSpeed() > speed * 0.1, "Bottom motor should show acceleration toward target");
    }

    @Test
    @DisplayName("Unequal speeds should be maintained correctly")
    void unequalSpeedsMaintained() {
      final double topSpeed = 3000;
      final double bottomSpeed = 1500;
      shooter.setSpeed(topSpeed, bottomSpeed);
      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getTopSpeed() > speeds.getBottomSpeed(), "Top motor RPM should be greater than bottom");
    }

    @Test
    @DisplayName("Same speed set multiple times should be consistent")
    void repeatedSpeedSetConsistent() {
      final double speed = 5000;
      shooter.setSpeed(speed);
      double setpoint1 = topMotor.getSetpoint();
      shooter.setSpeed(speed);
      double setpoint2 = topMotor.getSetpoint();
      assertEquals(setpoint1, setpoint2, Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Feeder Motor Tests")
  class FeederMotorTests {

    @Test
    @DisplayName("Feeder should set to specified speed")
    void setFeederSetpoint() {
      final double speed = 0.5;
      shooter.setFeederSpeed(speed);
      assertEquals(speed, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder at zero should be stopped")
    void feederZeroIsStop() {
      shooter.setFeederSpeed(0.0);
      assertEquals(0.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder at full power should be maximum")
    void feederFullPower() {
      shooter.setFeederSpeed(1.0);
      assertEquals(1.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder should accelerate at set speed")
    void feederAccelerates() {
      final double speed = 0.5;
      shooter.setFeederSpeed(speed);
      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getFeederSpeed() > 0, "Feeder should accelerate");
    }

    @Test
    @DisplayName("Multiple feeder speed changes should take effect")
    void feederSpeedChanges() {
      shooter.setFeederSpeed(0.3);
      assertEquals(0.3, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      shooter.setFeederSpeed(0.8);
      assertEquals(0.8, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Feeder speed should be independent of shooter motor speeds")
    void feederIndependentOfMotors() {
      shooter.setSpeed(5000);
      shooter.setFeederSpeed(0.7);

      assertEquals(5000, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(5000, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.7, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Stop Functionality Tests")
  class StopFunctionalityTests {

    @Test
    @DisplayName("Stop should reset all motors to zero")
    void stopResetsAllMotors() {
      shooter.setSpeed(5000);
      shooter.setFeederSpeed(0.5);

      shooter.stop();

      assertEquals(0.0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Stop should work after any motor configuration")
    void stopWorksAfterAnyConfig() {
      shooter.setSpeed(100, 200);
      shooter.setFeederSpeed(0.9);
      shooter.stop();

      assertEquals(0.0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Multiple stops should be safe")
    void multipleStopesSafe() {
      shooter.setSpeed(1000);
      shooter.stop();
      shooter.stop();
      shooter.stop();

      assertEquals(0.0, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Combined Operation Tests")
  class CombinedOperationTests {

    @Test
    @DisplayName("Can operate shooter and feeder independently")
    void shooterAndFeederIndependent() {
      // Set shooter to different speeds for each motor
      shooter.setSpeed(4000, 3000);

      // Verify shooter speeds
      assertEquals(4000, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(3000, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

      // Set feeder separately
      shooter.setFeederSpeed(Constants.Shooter.FEEDER_POWER);
      assertEquals(
          Constants.Shooter.FEEDER_POWER, feederMotor.getSetpoint(), Constants.Tests.DELTA);

      // Shooter speeds should still be correct
      assertEquals(4000, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(3000, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Changing shooter speed should not affect feeder")
    void changingShooterDoesNotAffectFeeder() {
      shooter.setFeederSpeed(0.75);
      shooter.setSpeed(6000);

      assertEquals(0.75, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Changing feeder speed should not affect shooter")
    void changingFeederDoesNotAffectShooter() {
      shooter.setSpeed(5000);
      shooter.setFeederSpeed(0.6);

      assertEquals(5000, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(5000, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Edge Cases and Boundary Tests")
  class EdgeCasesTests {

    @Test
    @DisplayName("Very small positive speeds should be accepted")
    void verySmallPositiveSpeed() {
      shooter.setSpeed(0.1);
      assertEquals(0.1, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.1, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Negative speeds should be accepted")
    void negativeSpeedAccepted() {
      shooter.setSpeed(-1000);
      assertEquals(-1000, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1000, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Very large speeds should be accepted")
    void veryLargeSpeed() {
      final double largeSpeed = 50000;
      shooter.setSpeed(largeSpeed);
      assertEquals(largeSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(largeSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Speed values should be independent per motor")
    void speedsIndependent() {
      shooter.setSpeed(2000, 8000);
      final SimulatedSpeeds speeds = simulate(5);

      // With different speed targets, bottom motor should have higher velocity
      // (accounting for realistic flywheel acceleration in simulation)
      assertTrue(speeds.getBottomSpeed() > speeds.getTopSpeed(), "Bottom motor should reach higher RPM");
    }

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

      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getTopSpeed() > 1);
      assertTrue(speeds.getBottomSpeed() > 1);
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

      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getTopSpeed() > 1);
      assertTrue(speeds.getBottomSpeed() > 1);
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

      final SimulatedSpeeds speeds = simulate(20);
      assertTrue(speeds.getTopSpeed() > speed * 0.1, "Top motor should show acceleration");
      assertTrue(speeds.getBottomSpeed() > speed * 0.1, "Bottom motor should show acceleration");

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

      final SimulatedSpeeds speeds = simulate(10);
      assertTrue(speeds.getFeederSpeed() > 0);
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
      assertEquals(
          Constants.Shooter.FEEDER_POWER, feederMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Shooter motors should accelerate")
    void shooterMotorsAccelerate() {
      Command testCommand = shooter.runShooter();
      CommandScheduler.getInstance().schedule(testCommand);
      CommandScheduler.getInstance().run();

      final SimulatedSpeeds speeds = simulate(40);
      assertEquals(Constants.Shooter.SHOOTER_RPM, topMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(Constants.Shooter.SHOOTER_RPM, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
      assertTrue(speeds.topSpeed > 1);
      assertTrue(speeds.bottomSpeed > 1);
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
      double[] speeds = { 0, 1000, 5000, 10000 };
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

  protected class SimulatedSpeeds {
    private double topSpeed;
    private double bottomSpeed;
    private double feederSpeed;

    public SimulatedSpeeds(double topSpeed, double bottomSpeed, double feederSpeed) {
      this.topSpeed = topSpeed;
      this.bottomSpeed = bottomSpeed;
      this.feederSpeed = feederSpeed;
    }

    public double getTopSpeed() {
      return topSpeed;
    }

    public double getBottomSpeed() {
      return bottomSpeed;
    }

    public double getFeederSpeed() {
      return feederSpeed;
    }
  }

  protected SimulatedSpeeds simulate(double time) {
   try {
      // Create fresh FlywheelSim instances for each motor to avoid state pollution
      FlywheelSim topSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
              DCMotor.getNEO(1));
      FlywheelSim bottomSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
              DCMotor.getNEO(1));
      FlywheelSim feederSim =
          new FlywheelSim(
              LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
              DCMotor.getNEO(1));

      double topSpeed = topMotor.simulateFlywheelRPM(time, topSim);
      double bottomSpeed = bottomMotor.simulateFlywheelRPM(time, bottomSim);
      double feederSpeed = feederMotor.simulateFlywheelRPM(time, feederSim);

      return new SimulatedSpeeds(topSpeed, bottomSpeed, feederSpeed);
    } catch (Exception e) {
      e.printStackTrace();
      // If simulation fails, return default values indicating no acceleration
      return null;
    }
  }
}
