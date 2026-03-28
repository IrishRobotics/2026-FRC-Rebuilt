package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.RunFeeder;

public class Feeder extends SubsystemBase implements AutoCloseable {
    private final SparkMax motor =
            new SparkMax(Constants.Feeder.FEEDER_MOTOR, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder motorEncoder =
            motor.getEncoder();

    public Feeder() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.encoder.velocityConversionFactor(1);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Stops the motor */
    public void stop() {
        motor.stopMotor();
    }
    /**
     * Sets the speed of the feeder motor
     *
     * @param speed the precent power of the motor
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Gets the speed in RPM of the motor
     */
    public double getSpeed() {
        return motorEncoder.getVelocity();
    }

    public Command run(double speed) {
        return new RunFeeder(this, speed);
    }

    @Override
    public void close() {
        motor.close();
    }
}
