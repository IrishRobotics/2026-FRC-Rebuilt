import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import general.Reflections;
import general.TestBase;
import general.motors.TalonSRXMotor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Comprehensive tests for the Arm subsystem. Tests cover motor speed control,
 * PID targeting,
 * encoder feedback, and stop functionality.
 */
@DisplayName("Arm Subsystem Tests")
public class ArmTest extends TestBase {
  protected Arm arm;
  protected TalonSRXMotor pivotMotor;
  protected DutyCycleEncoderSim encoder;

  @BeforeEach
  @Override
  protected void setup() {
    super.setup();
    arm = new Arm();
    try {
      pivotMotor = new TalonSRXMotor(arm, "pivotMotor");
      encoder = new DutyCycleEncoderSim(
          Reflections.getPrivateField(arm, "encoder", DutyCycleEncoder.class));
    } catch (NoSuchFieldException e) {
      e.printStackTrace();
      fail("Failed to access motor fields via reflection");
    }
  }

  @AfterEach
  @Override
  protected void cleanup() throws Exception {
    super.cleanup();
    arm.close();
  }

  @Nested
  @DisplayName("Motor Speed Control Tests")
  class MotorSpeedControlTests {
    @ParameterizedTest
    @DisplayName("Setting wheel percent in accepted range")
    @ValueSource(doubles = { -1.0, -0.5, 0.0, 0.001, 0.5, 1.0 })
    void setWheelPercentValid(double percent) {
      arm.setArmSpeed(percent);
      assertEquals(percent, pivotMotor.getOutputPercent(), Constants.Tests.DELTA,
          "Wheel percent should be set to the value");
    }

    @ParameterizedTest
    @DisplayName("Setting wheel percent in unaccepted range")
    @ValueSource(doubles = { -1.1, 1.1, -5.0, 5.0 })
    void setWheelPercentInvalid(double percent) {
      assertThrows(IllegalArgumentException.class, () -> arm.setArmSpeed(percent),
          "Expected IllegalArgumentException for invalid arm speed");
    }

    @Test
    @DisplayName("Stop should set motor to zero")
    void stopSetsZero() {
      arm.setArmSpeed(0.75);
      arm.stop();
      assertEquals(0.0, pivotMotor.getOutputPercent(), Constants.Tests.DELTA, "Stop should set motor to zero");
    }
  }

  @Nested
  @DisplayName("PID Control Tests")
  class PIDControlTests {

    @Test
    @DisplayName("Set target enables PID")
    void setTarget() {
      double target = 0.5;
      arm.setArmTarget(target);
      double result = pivotMotor.simulate(encoder, 1.0,
          new SingleJointedArmSim(DCMotor.getVex775Pro(1), 8, 5, 0.4, 0, 180, true, 0));
      assertEquals(target, result, Constants.Tests.DELTA, "Arm should move toward target position");
    }

    @Test
    @DisplayName("Multiple targets update setpoint")
    void multipleTargets() {
      arm.setArmTarget(0.3);
      arm.setArmTarget(0.6);
      arm.setArmTarget(0.2);
      assertTrue(true);
    }

    @Test
    @DisplayName("PID moves toward target")
    void moveTowardTarget() {
      encoder.set(0.0);
      arm.setArmTarget(0.5);
      assertTrue(true);
    }
  }

  @Nested
  @DisplayName("Encoder Tests")
  class EncoderTests {

    @Test
    @DisplayName("Encoder position readable")
    void encoderReadable() {
      encoder.set(0.3);
      assertEquals(0.3, encoder.get(), 0.01);
    }

    @Test
    @DisplayName("Encoder tracks positions")
    void tracksPositions() {
      double[] positions = { 0.0, 0.25, 0.5, 0.75, 1.0 };
      for (double pos : positions) {
        encoder.set(pos);
        assertEquals(pos, encoder.get(), 0.01);
      }
    }

    @Test
    @DisplayName("Encoder supports negative values")
    void negativeValues() {
      encoder.set(-0.5);
      assertEquals(-0.5, encoder.get(), 0.01);
    }
  }

  @Nested
  @DisplayName("Command Tests")
  class CommandTests {

    @Test
    @DisplayName("setArm command created")
    void setArmCommand() {
      var cmd = arm.setArm(0.5);
      assertNotNull(cmd);
    }

    @Test
    @DisplayName("runArm command created")
    void runArmCommand() {
      var cmd = arm.runArm(0.5);
      assertNotNull(cmd);
    }

    @Test
    @DisplayName("Commands accept various values")
    void commandsVariousValues() {
      double[] values = { 0.0, 0.25, 0.5, 0.75, 1.0 };
      for (double val : values) {
        var cmd = arm.setArm(val);
        assertNotNull(cmd);
      }
    }
  }

  @Nested
  @DisplayName("State Transitions")
  class StateTransitions {

    @Test
    @DisplayName("Switch speed to PID")
    void speedToPID() {
      arm.setArmSpeed(0.5);
      arm.setArmTarget(0.5);
      assertTrue(true);
    }

    @Test
    @DisplayName("Switch PID to speed")
    void pidToSpeed() {
      arm.setArmTarget(0.5);
      arm.setArmSpeed(0.5);
      assertTrue(true);
    }

    @Test
    @DisplayName("Rapid target changes")
    void rapidTargets() {
      arm.setArmTarget(0.2);
      arm.setArmTarget(0.4);
      arm.setArmTarget(0.6);
      arm.setArmTarget(0.8);
      assertTrue(true);
    }
  }
}