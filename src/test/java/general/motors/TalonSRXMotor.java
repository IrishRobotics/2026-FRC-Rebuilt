package general.motors;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import general.Reflections;

public class TalonSRXMotor {
    TalonSRXSimCollection motorSim;

    public TalonSRXMotor(Object obj, String propertyName) throws NoSuchFieldException {
        TalonSRX talonSRX = Reflections.getPrivateField(obj, propertyName, TalonSRX.class);
        motorSim = talonSRX.getSimCollection();
    }

    public double simulate(DutyCycleEncoderSim encoder, double time, SingleJointedArmSim sim) {
        final double dt = 0.02;
        final int steps = (int) (time / dt);
        for (int i = 0; i < steps; i++) {
            motorSim.setBusVoltage(12.0);
            encoder.set(Units.radiansToRotations(sim.getAngleRads()));
            CommandScheduler.getInstance().run();

            sim.setInputVoltage(motorSim.getMotorOutputLeadVoltage());
            sim.update(dt);
        }

        return sim.getAngleRads();
    }
}
