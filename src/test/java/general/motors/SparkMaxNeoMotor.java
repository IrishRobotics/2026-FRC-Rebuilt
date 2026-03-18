package general.motors;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import general.Reflections;

public class SparkMaxNeoMotor implements Motor {
  protected SparkMaxSim motorSim;

  public SparkMaxNeoMotor(Object obj, String propertyName) throws NoSuchFieldException {
    motorSim =
        new SparkMaxSim(
            Reflections.getPrivateField(obj, propertyName, SparkMax.class), DCMotor.getNEO(1));
  }

  @Override
  public void enable(boolean enabled) {
    if (enabled) {
      motorSim.enable();
    } else {
      motorSim.disable();
    }
  }

  @Override
  public double getSetpoint() {
    return motorSim.getSetpoint();
  }

  @Override
  public double simulateFlywheelRPM(double time, double JKgMetersSquared) {
    FlywheelSim sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), JKgMetersSquared, 1),
            DCMotor.getNEO(1));

    enable(true);
    final double dt = 0.02;
    final int steps = (int) (time / dt);
    for (int i = 0; i < steps; i++) {
      CommandScheduler.getInstance().run();

      motorSim.iterate(sim.getAngularVelocityRPM(), 12.0, dt);
      sim.setInputVoltage(motorSim.getAppliedOutput() * 12.0);
      sim.update(dt);
    }

    enable(false);
    return sim.getAngularVelocityRPM();
  }
}
