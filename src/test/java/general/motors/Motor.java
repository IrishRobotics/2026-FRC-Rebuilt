package general.motors;

public interface Motor {
  void enable(boolean enabled);

  double getSetpoint();

  double simulateFlywheelRPM(double time, double JKgMetersSquared);
}
