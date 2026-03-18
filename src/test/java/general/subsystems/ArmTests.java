package general.subsystems;

import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import general.Reflections;
import general.motors.TalonSRXMotor;

public class ArmTests {
    protected Arm arm;
    protected TalonSRXMotor pivotMotor;
    protected DutyCycleEncoderSim encoder;

    @BeforeEach
    protected void setup() {
        assert HAL.initialize(500, 0);

        arm = new Arm();

        try {
            pivotMotor = new TalonSRXMotor(arm, "pivotMotor");
            encoder = new DutyCycleEncoderSim(Reflections.getPrivateField(arm, "encoder", DutyCycleEncoder.class));
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
            fail("Failed to access motor fields via reflection");
        }
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    protected void shutdown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        arm.close();
    }
}
