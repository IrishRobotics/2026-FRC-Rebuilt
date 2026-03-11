package subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import general.ShooterTests;

import org.junit.jupiter.api.Test;

class ShooterTest extends ShooterTests {
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

  @Test
  void setFeederSetpoint() {
    final double speed = 0.5;
    shooter.setFeederSpeed(speed);
    assertEquals(speed, feederMotor.getSetpoint(), Constants.Tests.DELTA);
  }
}