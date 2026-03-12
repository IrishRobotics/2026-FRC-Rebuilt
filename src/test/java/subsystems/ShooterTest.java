package subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import general.subsystems.ShooterTests;

import org.junit.jupiter.api.Test;

class ShooterTest extends ShooterTests {
  @Test
  void setBothSetpoints() {
    final double speed = 10;
    shooter.setSpeed(speed);
    assertEquals(speed, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(speed, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final double[] speeds = simulate(10);
    assertTrue(speeds[0] > 1);
    assertTrue(speeds[1] > 1);
  }

  @Test
  void setBothSetpointsSeparate() {
    shooter.setSpeed(50, 25);
    assertEquals(50, topMotor.getSetpoint(), Constants.Tests.DELTA);
    assertEquals(25, bottomMotor.getSetpoint(), Constants.Tests.DELTA);

    final double[] speeds = simulate(10);
    assertTrue(speeds[0] > 1);
    assertTrue(speeds[1] > 1);
  }

  @Test
  void setFeederSetpoint() {
    final double speed = 0.5;
    shooter.setFeederSpeed(speed);
    assertEquals(speed, feederMotor.getSetpoint(), Constants.Tests.DELTA);
  }
}