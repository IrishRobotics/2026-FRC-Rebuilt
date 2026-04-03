// package subsystems;

// import static org.junit.jupiter.api.Assertions.assertEquals;
// import static org.junit.jupiter.api.Assertions.assertThrows;
// import static org.junit.jupiter.api.Assertions.assertTrue;
// import static org.junit.jupiter.api.Assertions.fail;

// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.hal.HAL;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.Drivetrain;
// import java.lang.reflect.Field;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// class DrivetrainTest {
//   private Drivetrain drivetrain;
//   private SparkMaxSim frontLeftMotor;
//   private SparkMaxSim frontRightMotor;
//   private SparkMaxSim backLeftMotor;
//   private SparkMaxSim backRightMotor;

//   @BeforeEach
//   void setup() {
//     assert HAL.initialize(500, 0);
//     drivetrain = new Drivetrain();
//     drivetrain.setSpeed(1);

//     try {
//       final Field frontLeftField = drivetrain.getClass().getDeclaredField("frontLeftMotor");
//       frontLeftField.setAccessible(true);
//       frontLeftMotor =
//           new SparkMaxSim((SparkMax) frontLeftField.get(drivetrain), DCMotor.getNEO(1));
//       final Field frontRightField = drivetrain.getClass().getDeclaredField("frontRightMotor");
//       frontRightField.setAccessible(true);
//       frontRightMotor =
//           new SparkMaxSim((SparkMax) frontRightField.get(drivetrain), DCMotor.getNEO(1));
//       final Field backLeftField = drivetrain.getClass().getDeclaredField("backLeftMotor");
//       backLeftField.setAccessible(true);
//       backLeftMotor = new SparkMaxSim((SparkMax) backLeftField.get(drivetrain),
// DCMotor.getNEO(1));
//       final Field backRightField = drivetrain.getClass().getDeclaredField("backRightMotor");
//       backRightField.setAccessible(true);
//       backRightMotor =
//           new SparkMaxSim((SparkMax) backRightField.get(drivetrain), DCMotor.getNEO(1));

//       final Field mecanumDrivetrainField = drivetrain.getClass().getDeclaredField("drive");
//       mecanumDrivetrainField.setAccessible(true);
//       ((MecanumDrive) mecanumDrivetrainField.get(drivetrain)).setDeadband(0);
//     } catch (NoSuchFieldException | IllegalAccessException e) {
//       e.printStackTrace();
//       fail("Failed to access motor fields via reflection");
//     }
//   }

//   @SuppressWarnings("PMD.SignatureDeclareThrowsException")
//   @AfterEach
//   void shutdown() throws Exception {
//     drivetrain.close();
//   }

//   @Test
//   void testDriveZero() {
//     drivetrain.drive(0, 0, 0);
//     assertEquals(0.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveForward() {
//     drivetrain.drive(1, 0, 0);
//     assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveBackward() {
//     drivetrain.drive(-1, 0, 0);
//     assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveLeft() {
//     drivetrain.drive(0, -1, 0);
//     assertEquals(-1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveRight() {
//     drivetrain.drive(0, 1, 0);
//     assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testTurnRight() {
//     drivetrain.drive(0, 0, 1);
//     assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(-1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveForwardRight() {
//     drivetrain.drive(1, 1, 0);
//     assertEquals(1.0, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.0, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.0, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(1.0, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveSpeed() {
//     drivetrain.setSpeed(0.5);
//     assertEquals(drivetrain.getSpeed(), 0.5, Constants.Tests.DELTA);
//     drivetrain.drive(1, 0, 0);
//     assertEquals(0.5, frontLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.5, frontRightMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.5, backLeftMotor.getSetpoint(), Constants.Tests.DELTA);
//     assertEquals(0.5, backRightMotor.getSetpoint(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testDriveSpeedException() {
//     Exception exception =
//         assertThrows(IllegalArgumentException.class, () -> drivetrain.setSpeed(1.5));

//     String expectedMessage = "Value 1.5 not in range [0.0, 1.0]";
//     String actualMessage = exception.getMessage();

//     assertTrue(actualMessage.contains(expectedMessage));

//     exception = assertThrows(IllegalArgumentException.class, () ->
// drivetrain.setSpeed(-5.00001));

//     expectedMessage = "Value -5.0 not in range [0.0, 1.0]";
//     actualMessage = exception.getMessage();

//     assertTrue(actualMessage.contains(expectedMessage));
//   }

//   @Test
//   void testToggleSpeedHighToLow() {
//     drivetrain.setSpeed(Constants.Drivetrain.HIGH_SPEED);
//     assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);

//     InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
//     cmd.initialize();

//     assertEquals(Constants.Drivetrain.LOW_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testToggleSpeedLowToHigh() {
//     drivetrain.setSpeed(Constants.Drivetrain.LOW_SPEED);
//     assertEquals(Constants.Drivetrain.LOW_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);

//     InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
//     cmd.initialize();

//     assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
//   }

//   @Test
//   void testToggleSpeedUndefinedToHigh() {
//     drivetrain.setSpeed(0);
//     assertEquals(0, drivetrain.getSpeed(), Constants.Tests.DELTA);

//     InstantCommand cmd = (InstantCommand) drivetrain.toggleSpeed();
//     cmd.initialize();

//     assertEquals(Constants.Drivetrain.HIGH_SPEED, drivetrain.getSpeed(), Constants.Tests.DELTA);
//   }
// }
