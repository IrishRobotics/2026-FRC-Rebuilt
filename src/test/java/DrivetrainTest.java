import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DrivetrainTest {
    private Drivetrain drivetrain;
    private SparkMaxSim frontLeftMotorSimulation;
    private SparkMaxSim frontRightMotorSimulation;
    private SparkMaxSim backLeftMotorSimulation;
    private SparkMaxSim backRightMotorSimulation;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        drivetrain = new Drivetrain();
        drivetrain.setSpeed(1);

        final SparkMax[] motors = drivetrain.testMode();
        frontLeftMotorSimulation = new SparkMaxSim(motors[0], DCMotor.getNEO(1));
        frontRightMotorSimulation = new SparkMaxSim(motors[1], DCMotor.getNEO(1));
        backLeftMotorSimulation = new SparkMaxSim(motors[2], DCMotor.getNEO(1));
        backRightMotorSimulation = new SparkMaxSim(motors[3], DCMotor.getNEO(1));
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        drivetrain.close();
    }

    @Test
    void testDriveZero() {
        drivetrain.drive(0, 0, 0);
        assertEquals(0.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveForward() {
        drivetrain.drive(1, 0, 0);
        assertEquals(1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveBackward() {
        drivetrain.drive(-1, 0, 0);
        assertEquals(-1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveLeft() {
        drivetrain.drive(0, -1, 0);
        assertEquals(-1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveRight() {
        drivetrain.drive(0, 1, 0);
        assertEquals(1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testTurnRight() {
        drivetrain.drive(0, 0, 1);
        assertEquals(1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(-1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveForwardRight() {
        drivetrain.drive(1, 1, 0);
        assertEquals(1.0, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.0, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.0, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(1.0, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveSpeed() {
        drivetrain.setSpeed(0.5);
        assertEquals(drivetrain.getSpeed(), 0.5, Constants.Tests.DELTA);
        drivetrain.drive(1, 0, 0);
        assertEquals(0.5, frontLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.5, frontRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.5, backLeftMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(0.5, backRightMotorSimulation.getSetpoint(), Constants.Tests.DELTA);
    }

    @Test
    void testDriveSpeedException() {
        Exception exception = assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(1.5));

        String expectedMessage = "Value 1.5 not in range [0.0, 1.0]";
        String actualMessage = exception.getMessage();

        assertTrue(actualMessage.contains(expectedMessage));

        exception = assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(-5.00001));

        expectedMessage = "Value -5.0 not in range [0.0, 1.0]";
        actualMessage = exception.getMessage();

        assertTrue(actualMessage.contains(expectedMessage));
    }

    @Test
    void testToggleSpeedHighToLow() {
        drivetrain.setSpeed(Constants.Drivetrain.highSpeed);
        assertEquals(Constants.Drivetrain.highSpeed, drivetrain.getSpeed(), Constants.Tests.DELTA);

        InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
        cmd.initialize();

        assertEquals(Constants.Drivetrain.lowSpeed, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }

    @Test
    void testToggleSpeedLowToHigh() {
        drivetrain.setSpeed(Constants.Drivetrain.lowSpeed);
        assertEquals(Constants.Drivetrain.lowSpeed, drivetrain.getSpeed(), Constants.Tests.DELTA);

        InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
        cmd.initialize();

        assertEquals(Constants.Drivetrain.highSpeed, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }
    
    @Test
    void testToggleSpeedUndefinedToHigh() {
        drivetrain.setSpeed(0);
        assertEquals(0, drivetrain.getSpeed(), Constants.Tests.DELTA);

        InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
        cmd.initialize();

        assertEquals(Constants.Drivetrain.highSpeed, drivetrain.getSpeed(), Constants.Tests.DELTA);
    }
}
