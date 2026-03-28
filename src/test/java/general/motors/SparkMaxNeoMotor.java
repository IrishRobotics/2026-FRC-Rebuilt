package general.motors;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import general.Reflections;

public class SparkMaxNeoMotor {
  protected SparkMaxSim motorSim;

  public SparkMaxNeoMotor(Object obj, String propertyName) throws NoSuchFieldException {
    motorSim =
        new SparkMaxSim(
            Reflections.getPrivateField(obj, propertyName, SparkMax.class), DCMotor.getNEO(1));
  }

  public void enable(boolean enabled) {
    if (enabled) {
      motorSim.enable();
    } else {
      motorSim.disable();
    }
  }

  public double getSetpoint() {
    return motorSim.getSetpoint();
  }

  public double simulateFlywheelRPM(double time, FlywheelSim sim) {
    boolean wasEnabled = false;
    try {
      enable(true);
      wasEnabled = true;
      final double dt = 0.02;
      final int steps = (int) (time / dt);

      for (int i = 0; i < steps; i++) {
        // Get the motor output and apply to the simulation
        double voltage = motorSim.getAppliedOutput() * 12.0;
        sim.setInputVoltage(voltage);
        sim.update(dt);

        // Feed the simulation velocity back to the motor
        motorSim.iterate(sim.getAngularVelocityRPM(), 12.0, dt);
      }

      return sim.getAngularVelocityRPM();
    } catch (Exception e) {
      // Log but continue - simulation state issues should not crash
      e.printStackTrace();
      return 0.0;
    } finally {
      // Ensure motor is disabled regardless of outcome
      if (wasEnabled) {
        try {
          enable(false);
        } catch (Exception e) {
          // Ignore errors during cleanup
        }
      }
    }
  }
}
