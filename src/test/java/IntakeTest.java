

import static org.junit.jupiter.api.Assertions.*;

import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import general.TestBase;
import general.motors.TalonSRXMotor;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

/**
 * Comprehensive tests for the Intake subsystem. Tests cover motor control and
 * stop functionality.
 */
@DisplayName("Intake Subsystem Tests")
class IntakeTest extends TestBase {
  private Intake intake;
  private TalonSRXMotor intakeMotor;

  @BeforeEach
  @Override
  protected void setup() {
    super.setup();
    intake = new Intake();

    try {
      intakeMotor = new TalonSRXMotor(intake, "intakeMotor");
    } catch (NoSuchFieldException e) {
      e.printStackTrace();
      fail("Failed to access wheel motor field via reflection");
    }
  }

  @AfterEach
  @Override
  protected void cleanup() throws Exception {
    super.cleanup();
    intake.close();
  }

  @Nested
  @DisplayName("Motor Speed Control Tests")
  class MotorSpeedControlTests {
    @ParameterizedTest
    @DisplayName("Setting wheel percent in accepted range")
    @ValueSource(doubles = { -1.0, -0.5, 0.0, 0.001, 0.5, 1.0 })
    void setWheelPercentValid(double percent) {
      intake.setWheelPercent(percent);
      assertEquals(percent, intakeMotor.getOutput(), Constants.Tests.DELTA, "Wheel percent should be set to the value");
    }

    @ParameterizedTest
    @DisplayName("Setting wheel percent in unaccepted range")
    @ValueSource(doubles = { -1.1, 1.1, -5.0, 5.0 })
    void setWheelPercentInvalid(double percent) {
      assertThrows(IllegalArgumentException.class, () -> intake.setWheelPercent(percent),
          "Expected IllegalArgumentException for invalid wheel percent");
    }

    @Test
    @DisplayName("Stop should set motor to zero")
    void stopSetsZero() {
      intake.setWheelPercent(0.75);
      intake.stop();
      assertEquals(0.0, intakeMotor.getOutput(), Constants.Tests.DELTA, "Stop should set motor to zero");
    }
  }

  @Nested
  @DisplayName("Command Tests")
  class CommandTests {

    @ParameterizedTest
    @DisplayName("runIntake command should be created successfully in range")
    @ValueSource(doubles = { -1.0, -0.5, 0.0, 0.001, 0.5, 1.0 })
    void runIntakeCommandCreated(double percent) {
      var cmd = intake.runIntake(percent);
      assertNotNull(cmd, "runIntake command should be created");
    }

    @ParameterizedTest
    @DisplayName("runIntake command should throw exception for out of range")
    @ValueSource(doubles = { -1.1, 1.1, -5.0, 5.0 })
    void runIntakeCommandInvalid(double percent) {
      assertThrows(IllegalArgumentException.class, () -> intake.runIntake(percent),
          "Expected IllegalArgumentException for invalid runIntake percent");
    }

    @ParameterizedTest
    @DisplayName("runIntake command should set motor percent when executed")
    @ValueSource(doubles = { -1.0, -0.5, 0.0, 0.001, 0.5, 1.0 })
    void runIntakeCommandExecution(double percent) {
      var cmd = intake.runIntake(percent);
      cmd.initialize();
      assertEquals(percent, intakeMotor.getOutput(), Constants.Tests.DELTA, "runIntake command should set motor percent when executed");
    }
  }
}
