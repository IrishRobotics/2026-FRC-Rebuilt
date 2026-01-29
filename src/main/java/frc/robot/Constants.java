package frc.robot;

public final class Constants {
  private Constants() {}

  public static class Control {
    private Control() {}

    public static final int driverControllerPort = 0;
    public static final int coopControllerPort = 1;
  }

  public static class Drivetrain {
    private Drivetrain() {}

    public static final int frontLeft = 1;
    public static final int frontRight = 2;
    public static final int backLeft = 3;
    public static final int backRight = 4;

    public static final double highSpeed = 0.75;
    public static final double lowSpeed = 0.5;
  }

  public static class Tests {
    private Tests() {}

    public static final double DELTA = 1e-2;
  }
}
