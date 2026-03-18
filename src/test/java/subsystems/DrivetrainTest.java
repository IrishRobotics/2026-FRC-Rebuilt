package subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import general.TestBase;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Comprehensive tests for the Drivetrain subsystem.
 * Tests cover all movement directions, speed control, and edge cases.
 */
@DisplayName("Drivetrain Tests")
class DrivetrainTest extends TestBase {
  private Drivetrain drivetrain;
  private SparkMaxSim frontLeftMotor;
  private SparkMaxSim frontRightMotor;
  private SparkMaxSim backLeftMotor;
  private SparkMaxSim backRightMotor;

  @BeforeEach
  @Override
  protected void setup() {
    super.setup();
    drivetrain = new Drivetrain();
    drivetrain.setSpeed(1);

    try {
      final Field frontLeftField = drivetrain.getClass().getDeclaredField("frontLeftMotor");
      frontLeftField.setAccessible(true);
      frontLeftMotor =
          new SparkMaxSim((SparkMax) frontLeftField.get(drivetrain), DCMotor.getNEO(1));

      final Field frontRightField = drivetrain.getClass().getDeclaredField("frontRightMotor");
      frontRightField.setAccessible(true);
      frontRightMotor =
          new SparkMaxSim((SparkMax) frontRightField.get(drivetrain), DCMotor.getNEO(1));

      final Field backLeftField = drivetrain.getClass().getDeclaredField("backLeftMotor");
      backLeftField.setAccessible(true);
      backLeftMotor = new SparkMaxSim((SparkMax) backLeftField.get(drivetrain), DCMotor.getNEO(1));

      final Field backRightField = drivetrain.getClass().getDeclaredField("backRightMotor");
      backRightField.setAccessible(true);
      backRightMotor =
          new SparkMaxSim((SparkMax) backRightField.get(drivetrain), DCMotor.getNEO(1));

      final Field mecanumDrivetrainField = drivetrain.getClass().getDeclaredField("drive");
      mecanumDrivetrainField.setAccessible(true);
      ((MecanumDrive) mecanumDrivetrainField.get(drivetrain)).setDeadband(0);
    } catch (NoSuchFieldException | IllegalAccessException e) {
      fail("Failed to access motor fields via reflection: " + e.getMessage());
    }
  }

  @AfterEach
  protected void shutdown() {
    cleanup();
    if (drivetrain != null) {
      drivetrain.close();
    }
  }

  @Nested
  @DisplayName("Basic Movement Tests")
  class BasicMovementTests {

    @Test
    @DisplayName("Drive with zero inputs should stop all motors")
    void testDriveZero() {
      drivetrain.drive(0, 0, 0);
      assertEquals(0.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive forward should move all motors forward equally")
    void testDriveForward() {
      drivetrain.drive(1, 0, 0);
      assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive backward should move all motors backward equally")
    void testDriveBackward() {
      drivetrain.drive(-1, 0, 0);
      assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive left should strafe with correct motor directions")
    void testDriveLeft() {
      drivetrain.drive(0, -1, 0);
      assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive right should strafe with correct motor directions")
    void testDriveRight() {
      drivetrain.drive(0, 1, 0);
      assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Turn right should rotate motors correctly")
    void testTurnRight() {
      drivetrain.drive(0, 0, 1);
      assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Turn left should rotate motors correctly")
    void testTurnLeft() {
      drivetrain.drive(0, 0, -1);
      assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Combined Movement Tests")
  class CombinedMovementTests {

    @Test
    @DisplayName("Drive forward and right should blend movements correctly")
    void testDriveForwardRight() {
      drivetrain.drive(1, 1, 0);
      assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive forward and left should blend movements correctly")
    void testDriveForwardLeft() {
      drivetrain.drive(1, -1, 0);
      assertEquals(0.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive backward and right should blend movements correctly")
    void testDriveBackwardRight() {
      drivetrain.drive(-1, 1, 0);
      assertEquals(0.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Drive backward and left should blend movements correctly")
    void testDriveBackwardLeft() {
      drivetrain.drive(-1, -1, 0);
      assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Speed Control Tests")
  class SpeedControlTests {

    @Test
    @DisplayName("Speed 0.5 should scale all motor outputs by 0.5")
    void testDriveSpeedHalf() {
      drivetrain.setSpeed(0.5);
      assertEquals(0.5, drivetrain.getSpeed(), Constants.Tests.DELTA);
      drivetrain.drive(1, 0, 0);
      assertEquals(0.5, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.5, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.5, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.5, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Speed 0.0 should stop all motors")
    void testDriveSpeedZero() {
      drivetrain.setSpeed(0.0);
      assertEquals(0.0, drivetrain.getSpeed(), Constants.Tests.DELTA);
      drivetrain.drive(1, 0, 0);
      assertEquals(0.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(0.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Speed 1.0 should output full power")
    void testDriveSpeedFull() {
      drivetrain.setSpeed(1.0);
      assertEquals(1.0, drivetrain.getSpeed(), Constants.Tests.DELTA);
      drivetrain.drive(1, 0, 0);
      assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
      assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Speed can be changed multiple times")
    void testDriveSpeedChanges() {
      drivetrain.setSpeed(0.25);
      drivetrain.drive(1, 0, 0);
      assertEquals(0.25, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);

      drivetrain.setSpeed(0.75);
      drivetrain.drive(1, 0, 0);
      assertEquals(0.75, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
    }
  }

  @Nested
  @DisplayName("Speed Validation Tests")
  class SpeedValidationTests {

    @Test
    @DisplayName("Speed > 1.0 should throw IllegalArgumentException")
    void testSetSpeedAboveMax() {
      assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(1.5));
    }

    @Test
    @DisplayName("Speed < 0.0 should throw IllegalArgumentException")
    void testSetSpeedBelowMin() {
      assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(-0.1));
    }

    @Test
    @DisplayName("Speed 1.0 should be valid")
    void testSetSpeedMax() {
      assertDoesNotThrow(() -> drivetrain.setSpeed(1.0));
      assertEquals(1.0, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Speed 0.0 should be valid")
    void testSetSpeedMin() {
      assertDoesNotThrow(() -> drivetrain.setSpeed(0.0));
      assertEquals(0.0, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Negative speed with large magnitude should throw")
    void testSetSpeedLargeNegative() {
      assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(-5.0));
    }
  }

  @Nested
  @DisplayName("Toggle Speed Tests")
  class ToggleSpeedTests {

    @Test
    @DisplayName("Toggle from HIGH_SPEED to LOW_SPEED")
    void testToggleSpeedHighToLow() {
      drivetrain.setSpeed(Constants.Drivetrain.HIGH_SPEED);
      assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);

      var cmd = drivetrain.toggleSpeed();
      cmd.initialize();

      assertEquals(Constants.Drivetrain.LOW_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Toggle from LOW_SPEED to HIGH_SPEED")
    void testToggleSpeedLowToHigh() {
      drivetrain.setSpeed(Constants.Drivetrain.LOW_SPEED);
      assertEquals(Constants.Drivetrain.LOW_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);

      var cmd = drivetrain.toggleSpeed();
      cmd.initialize();

      assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Toggle from intermediate speed should go to HIGH_SPEED")
    void testToggleSpeedIntermediateToHigh() {
      drivetrain.setSpeed(0.5);
      assertEquals(0.5, drivetrain.getSpeed(), Constants.Tests.DELTA);

      var cmd = drivetrain.toggleSpeed();
      cmd.initialize();

      assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Toggle from zero should go to HIGH_SPEED")
    void testToggleSpeedZeroToHigh() {
      drivetrain.setSpeed(0.0);
      assertEquals(0.0, drivetrain.getSpeed(), Constants.Tests.DELTA);

      var cmd = drivetrain.toggleSpeed();
      cmd.initialize();

      assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    @DisplayName("Repeated toggles should alternate correctly")
    void testRepeatedToggles() {
      drivetrain.setSpeed(Constants.Drivetrain.HIGH_SPEED);

      for (int i = 0; i < 4; i++) {
        var cmd = drivetrain.toggleSpeed();
        cmd.initialize();

        if (i % 2 == 0) {
          assertEquals(Constants.Drivetrain.LOW_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
        } else {
          assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(),
              Constants.Tests.DELTA);
        }
      }
    }
  }

  @Nested
  @DisplayName("Edge Cases and Boundary Tests")
  class EdgeCasesTests {

    @Test
    @DisplayName("Motor output should never exceed 1.0")
    void testMotorOutputMaximum() {
      drivetrain.setSpeed(1.0);
      drivetrain.drive(1, 1, 1);

      assertTrue(Math.abs(frontLeftMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(frontRightMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(backLeftMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(backRightMotor.getSetpoint()) <= 1.0);
    }

    @Test
    @DisplayName("Motor output should never go below -1.0")
    void testMotorOutputMinimum() {
      drivetrain.setSpeed(1.0);
      drivetrain.drive(-1, -1, -1);

      assertTrue(Math.abs(frontLeftMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(frontRightMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(backLeftMotor.getSetpoint()) <= 1.0);
      assertTrue(Math.abs(backRightMotor.getSetpoint()) <= 1.0);
    }
  }
}
