package general.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;
import general.motors.SparkMaxNeoMotor;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public abstract class ShooterTests {
  protected Shooter shooter;
  protected SparkMaxNeoMotor topMotor;
  protected SparkMaxNeoMotor bottomMotor;
  protected SparkMaxNeoMotor feederMotor;

  @BeforeEach
  void setup() {
    boolean halInitialized = HAL.initialize(500, 0);
    assertTrue(halInitialized, "HAL initialization failed");
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
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

  @AfterEach
  void shutdown() {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    shooter.close();
  }

  /**
   * Simulates both motors returning their speeds
   *
   * @return {top speed, bottom speed, feeder speed}
   */
  protected double[] simulate(double time) {
    try {
      // Create fresh FlywheelSim instances for each motor to avoid state pollution
      FlywheelSim topSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
          DCMotor.getNEO(1));
      FlywheelSim bottomSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
          DCMotor.getNEO(1));
      FlywheelSim feederSim = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1),
          DCMotor.getNEO(1));

      double topSpeed = topMotor.simulateFlywheelRPM(time, topSim);
      double bottomSpeed = bottomMotor.simulateFlywheelRPM(time, bottomSim);
      double feederSpeed = feederMotor.simulateFlywheelRPM(time, feederSim);

      return new double[] { topSpeed, bottomSpeed, feederSpeed };
    } catch (Exception e) {
      e.printStackTrace();
      // If simulation fails, return default values indicating no acceleration
      return new double[] { 0.0, 0.0, 0.0 };
    }
  }
}
