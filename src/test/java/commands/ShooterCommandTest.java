package commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShooterCommandTest {
  private Shooter shooter;
  private SparkMaxSim topMotor;
  private SparkMaxSim bottomMotor;

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

      topMotor = new SparkMaxSim((SparkMax) topMotorField.get(shooter), DCMotor.getNEO(1));
      bottomMotor = new SparkMaxSim((SparkMax) bottomMotorField.get(shooter), DCMotor.getNEO(1));
    } catch (NoSuchFieldException | IllegalAccessException e) {
      e.printStackTrace();
      fail("Failed to access motor fields via reflection");
    }
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    shooter.close();
  }

  /**
   * Simulates both motors returning their speeds
   *
   * @return {top speed, bottom speed}
   */
  AngularVelocity[] simulate(double time) {
    FlywheelSim topSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1), DCMotor.getNEO(1));
    FlywheelSim bottomSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.07609, 1), DCMotor.getNEO(1));

    topMotor.enable();
    bottomMotor.enable();

    final double dt = 0.02;
    final int steps = (int) (time / dt);
    for (int i = 0; i < steps; i++) {
      CommandScheduler.getInstance().run();

      topMotor.iterate(topSim.getAngularVelocityRPM(), 12.0, dt);
      bottomMotor.iterate(bottomSim.getAngularVelocityRPM(), 12.0, dt);

      topSim.setInputVoltage(topMotor.getAppliedOutput() * 12.0);
      bottomSim.setInputVoltage(bottomMotor.getAppliedOutput() * 12.0);

      topSim.update(dt);
      bottomSim.update(dt);
    }

    return new AngularVelocity[] {topSim.getAngularVelocity(), bottomSim.getAngularVelocity()};
  }

  @Test
  void setBothSetpoints() {
    final double speed = 10;
    Command testCommand = shooter.runAtSpeed(10);
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final AngularVelocity[] speeds = simulate(10);
    assertEquals(speed, speeds[0].abs(Units.RPM), 1);
    assertEquals(speed, speeds[1].abs(Units.RPM), 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
  }

  @Test
  void setBothSetpointsSeparate() {
    final double topSpeed = 50;
    final double bottomSpeed = 25;
    Command testCommand = shooter.runAtSpeed(topSpeed, bottomSpeed);
    CommandScheduler.getInstance().schedule(testCommand);
    CommandScheduler.getInstance().run();

    assertEquals(topSpeed, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(bottomSpeed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final AngularVelocity[] speeds = simulate(10);
    assertEquals(topSpeed, speeds[0].abs(Units.RPM), 1);
    assertEquals(bottomSpeed, speeds[1].abs(Units.RPM), 1);

    CommandScheduler.getInstance().cancelAll();
    assertEquals(0, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(0, bottomMotor.getSetpoint(), Constants.Tests.DELTA);
  }
}
