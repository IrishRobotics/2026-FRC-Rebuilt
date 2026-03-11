package general;

import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;

public abstract class ShooterTests {
  protected Shooter shooter;
  protected SparkMaxSim topMotor;
  protected SparkMaxSim bottomMotor;
  protected SparkMaxSim feederMotor;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    shooter = new Shooter();

    try {
      final Field topMotorField = shooter.getClass().getDeclaredField("topMotor");
      topMotorField.setAccessible(true);
      final Field bottomMotorField = shooter.getClass().getDeclaredField("bottomMotor");
      bottomMotorField.setAccessible(true);
      final Field feederMotorField = shooter.getClass().getDeclaredField("feederMotor");
      feederMotorField.setAccessible(true);

      topMotor = new SparkMaxSim((SparkMax) topMotorField.get(shooter), DCMotor.getNEO(1));
      bottomMotor = new SparkMaxSim((SparkMax) bottomMotorField.get(shooter), DCMotor.getNEO(1));
      feederMotor = new SparkMaxSim((SparkMax) feederMotorField.get(shooter), DCMotor.getNEO(1));
    } catch (NoSuchFieldException | IllegalAccessException e) {
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
  protected AngularVelocity[] simulate(double time) {
    FlywheelSim topSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1), DCMotor.getNEO(1));
    FlywheelSim bottomSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1), DCMotor.getNEO(1));
    FlywheelSim feederSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1), DCMotor.getNEO(1));

    topMotor.enable();
    bottomMotor.enable();
    feederMotor.enable();

    final double dt = 0.02;
    final int steps = (int) (time / dt);
    double[] data = new double[steps];
    for (int i = 0; i < steps; i++) {
      CommandScheduler.getInstance().run();
    
      topMotor.iterate(topSim.getAngularVelocityRPM(), 12.0, dt);
      bottomMotor.iterate(bottomSim.getAngularVelocityRPM(), 12.0, dt);
      feederMotor.iterate(feederSim.getAngularVelocityRPM(), 12.0, dt);

      topSim.setInputVoltage(topMotor.getAppliedOutput() * 12.0);
      bottomSim.setInputVoltage(bottomMotor.getAppliedOutput() * 12.0);
      feederSim.setInputVoltage(feederMotor.getAppliedOutput() * 12.0);

      topSim.update(dt);
      bottomSim.update(dt);
      feederSim.update(dt);

      data[i] = topSim.getAngularVelocityRPM();
    }

    return new AngularVelocity[] { topSim.getAngularVelocity(), bottomSim.getAngularVelocity(),
        feederSim.getAngularVelocity() };
  }
}
