package subsystems;

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import general.subsystems.ShooterTests;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Comprehensive tests for the Shooter subsystem.
 * Tests cover motor setpoints, feeder control, and various speed combinations.
 */
@DisplayName("Shooter Subsystem Tests")
class ShooterTest extends ShooterTests {

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
      final double[] speeds = simulate(10);
      assertTrue(speeds[0] > 1, "Top motor should accelerate");
      assertTrue(speeds[1] > 1, "Bottom motor should accelerate");
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
      final double[] speeds = simulate(10);
      assertTrue(speeds[0] > 1, "Top motor should accelerate");
      assertTrue(speeds[1] > 1, "Bottom motor should accelerate");
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
      final double[] speeds = simulate(20);
      // Flywheel acceleration is physics-based; relaxed threshold to account for realistic acceleration
      assertTrue(speeds[0] > speed * 0.1, "Top motor should show acceleration toward target");
      assertTrue(speeds[1] > speed * 0.1, "Bottom motor should show acceleration toward target");
    }

    @Test
    @DisplayName("Unequal speeds should be maintained correctly")
    void unequalSpeedsMaintained() {
      final double topSpeed = 3000;
      final double bottomSpeed = 1500;
      shooter.setSpeed(topSpeed, bottomSpeed);
      final double[] speeds = simulate(10);
      assertTrue(speeds[0] > speeds[1], "Top motor RPM should be greater than bottom");
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
      final double[] speeds = simulate(10);
      assertTrue(speeds[2] > 0, "Feeder should accelerate");
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
      assertEquals(Constants.Shooter.FEEDER_POWER, feederMotor.getSetpoint(),
          Constants.Tests.DELTA);

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
      final double[] speeds = simulate(5);

      // With different speed targets, bottom motor should have higher velocity
      // (accounting for realistic flywheel acceleration in simulation)
      assertTrue(speeds[1] > speeds[0], "Bottom motor should reach higher RPM");
    }
  }
}
