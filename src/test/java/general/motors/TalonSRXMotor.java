package general.motors;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import general.Reflections;

public class TalonSRXMotor {
  TalonSRXSimCollection motorSim;
  TalonSRX talonSRX;

  public TalonSRXMotor(Object obj, String propertyName) throws NoSuchFieldException {
    talonSRX = Reflections.getPrivateField(obj, propertyName, TalonSRX.class);
    motorSim = talonSRX.getSimCollection();
    motorSim.setBusVoltage(12.0);
  }

  public double getOutput() {
    return talonSRX.getMotorOutputPercent();
  }

  public double simulate(DutyCycleEncoderSim encoder, double time, SingleJointedArmSim sim) {
    motorSim.setBusVoltage(12.0);

    final double dt = 0.02;
    final int steps = (int) (time / dt);
    for (int i = 0; i < steps; i++) {
      encoder.set(Units.radiansToRotations(sim.getAngleRads()));
      CommandScheduler.getInstance().run();

      sim.setInputVoltage(motorSim.getMotorOutputLeadVoltage());
      sim.update(dt);
    }

    return sim.getAngleRads();
  }
}
