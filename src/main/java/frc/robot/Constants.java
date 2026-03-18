package frc.robot;

/** The Constants class holds all "Magic Numbers" and other constants */
public final class Constants {
  private Constants() {}

  /** Constants related to sensors */
  public static class Sensors {
    private Sensors() {}

    /** The CAN port of the Pigeon IMU */
    public static final int PIGEON_ID = 5;
  }

  /** Constants related to controlling the robot */
  public static class Control {
    private Control() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int COOP_CONTROLLER_PORT = 1;
  }

  /** Constants related to the drive system */
  public static class Drivetrain {
    private Drivetrain() {}

    public static final int FRONT_LEFT_MOTOR = 1;
    public static final int FRONT_RIGHT_MOTOR = 2;
    public static final int BACK_LEFT_MOTOR = 3;
    public static final int BACK_RIGHT_MOTOR = 4;

    public static final int IMU_ID = 20;

    /** The high-speed limit for drivetrain movement */
    public static final double HIGH_SPEED = 0.75;

    /** The low-speed limit for drivetrain movement */
    public static final double LOW_SPEED = 0.5;

    /** The width between the wheels across the robot */
    public static final double WHEEL_WIDTH = -1;

    /** The width between the wheels along the robot */
    public static final double WHEEL_LENGTH = -1;
  }

  public static class Arm {
    private Arm() {}

    public static final int ARM_MOTOR = 8;

    public static final double PID_P = 0.01;
    public static final double PID_I = 0;
    public static final double PID_D = 0;

    public static final int ENCODER_PORT = 0;
  }

  public static class Intake {
    private Intake() {}

    public static final int WHEEL_MOTOR = 9;

    /** The speed limit for intake movement */
    public static final double WHEEL_SPEED = 1;
  }

  public static class Shooter {
    private Shooter() {}

    public static final int TOP_MOTOR = 6;
    public static final int BOTTOM_MOTOR = 5;
    public static final int FEEDER_MOTOR = 7;

    public static final double SHOOTER_RPM = 7000;
    public static final double FEEDER_POWER = 0.5;
    public static final double FEEDER_WAIT = 0.5;

    public static final double PID_P = 0.00005;
    public static final double PID_I = 0.0000005;
    public static final double PID_D = 0.000005;
    // public static final double PID_P = 0.001;
    // public static final double PID_I = 0.00000005;
    // public static final double PID_D = 0.000000005;
  }

  /** This class holds constants used in tests */
  public static class Tests {
    private Tests() {}

    /** Maximum allowed deviation for tests using doubles */
    public static final double DELTA = 1e-2;
  }
}
