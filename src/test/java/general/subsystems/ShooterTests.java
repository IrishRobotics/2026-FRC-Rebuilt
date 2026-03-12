package general.subsystems;

import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;
import general.motors.Motor;
import general.motors.SparkMaxNeoMotor;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public abstract class ShooterTests {
  protected Shooter shooter;
  protected Motor topMotor;
  protected Motor bottomMotor;
  protected Motor feederMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
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
    double topSpeed = topMotor.simulateFlywheelRPM(time, 0.07609);
    double bottomSpeed = bottomMotor.simulateFlywheelRPM(time, 0.07609);
    double feederSpeed = feederMotor.simulateFlywheelRPM(time, 0.07609);

    return new double[] {topSpeed, bottomSpeed, feederSpeed};
  }
}
