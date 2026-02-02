package subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class ShooterTest {
    private Shooter shooter;
    private SparkMaxSim topMotor;
    private SparkMaxSim bottomMotor;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        shooter = new Shooter();

        try {
            final Field topMotorField = shooter.getClass().getDeclaredField("topMotor");
            topMotorField.setAccessible(true);
            final Field bottomMotorField = shooter.getClass().getDeclaredField("bottomMotor");
            bottomMotorField.setAccessible(true);

            topMotor = new SparkMaxSim((SparkMax) topMotorField.get(shooter), DCMotor.getNEO(0));
            bottomMotor = new SparkMaxSim((SparkMax) bottomMotorField.get(shooter), DCMotor.getNEO(0));
        } catch (NoSuchFieldException | IllegalAccessException e) {
            e.printStackTrace();
            fail("Failed to access motor fields via reflection");
        }

    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        shooter.close();
    }

    /**
     * Simulates both motors returning their speeds
     * 
     * @return {top speed, bottom speed}
     */
    AngularVelocity[] simulate(double time) {
        FlywheelSim topSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.07609, 1),
            DCMotor.getNEO(1));
        FlywheelSim bottomSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.07609, 1),
            DCMotor.getNEO(1));
        
        topMotor.enable();
        bottomMotor.enable();

        final double dt = 0.02;
        final int steps = (int) (time / dt);
        for (int i = 0; i < steps; i++) {
            topMotor.iterate(topSim.getAngularVelocityRPM(), 12.0, dt);
            bottomMotor.iterate(bottomSim.getAngularVelocityRPM(), 12.0, dt);
            
            topSim.setInputVoltage(topMotor.getAppliedOutput() * 12.0);
            bottomSim.setInputVoltage(bottomMotor.getAppliedOutput() * 12.0);
            
            topSim.update(dt);
            bottomSim.update(dt);
        }

        return new AngularVelocity[] { topSim.getAngularVelocity(), bottomSim.getAngularVelocity() };
    }

    @Test
    void setBothSetpoints() {
        final double speed = 10;
        shooter.setSpeed(speed);
        assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

        final AngularVelocity[] speeds = simulate(10);
        assertEquals(speed, speeds[0].abs(Units.RPM), 1);
        assertEquals(speed, speeds[1].abs(Units.RPM), 1);
    }

    @Test
    void setBothSetpointsSeparate() {
        shooter.setSpeed(50, 25);
        assertEquals(50, topMotor.getSetpoint(), Constants.Tests.DELTA);
        assertEquals(25, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

        final AngularVelocity[] speeds = simulate(10);
        assertEquals(50, speeds[0].abs(Units.RPM), 1);
        assertEquals(25, speeds[1].abs(Units.RPM), 1);
    }
}
