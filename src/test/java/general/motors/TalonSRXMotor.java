package general.motors;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import general.Reflections;
import general.TestBase;

public class TalonSRXMotor {
  TalonSRXSimCollection motorSim;
  TalonSRX motor;

  public TalonSRXMotor(Object obj, String propertyName) throws NoSuchFieldException {
    motor = Reflections.getPrivateField(obj, propertyName, TalonSRX.class);
    motorSim = motor.getSimCollection();
    motorSim.setBusVoltage(12.0);
  }

  public double getOutputPercent() {
    TestBase.sleep();
    return motor.getMotorOutputPercent();
  }

  public double getOutputVoltage() {
    TestBase.sleep();
    return motorSim.getMotorOutputLeadVoltage();
  }

  public double simulate(DutyCycleEncoderSim encoder, double time, SingleJointedArmSim sim) {
    motorSim.setBusVoltage(12.0);

    final double dt = 0.1;
    final int steps = (int) (time / dt);
      for (int i = 0; i < steps; i++) {
        CommandScheduler.getInstance().run();

        sim.setInputVoltage(getOutputVoltage());
        sim.update(dt);
        
        encoder.set(Units.radiansToRotations(sim.getAngleRads()));
      }

    return Units.radiansToRotations(sim.getAngleRads());
  }
}
