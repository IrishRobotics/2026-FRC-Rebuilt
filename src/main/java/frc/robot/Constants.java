package frc.robot;

/** The Constants class holds all "Magic Numbers" and other constants */
public final class Constants {
  private Constants() {}

  /** This class holds all constants related to sensors */
  public static class Sensors {
    private Sensors() {}

    /** The CAN port of the Pigeon IMU */
    public static final int PIGEON_ID = 5;
  }

  /** This class holds constants related to controlling the robot */
  public static class Control {
    private Control() {}

    /** This is the gamepad port for the driver's controller */
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /** This is the gamepad port for the co-driver's controller */
    public static final int COOP_CONTROLLER_PORT = 1;
  }

  /** This class holds constants related to the drive system */
  public static class Drivetrain {
    private Drivetrain() {}

    /** The motor id for the motor in the front left of the bot */
    public static final int FRONT_LEFT_MOTOR = 1;

    /** The motor id for the motor in the front right of the bot */
    public static final int FRONT_RIGHT_MOTOR = 2;

    /** The motor id for the motor in the back left of the bot */
    public static final int BACK_LEFT_MOTOR = 3;

    /** The motor id for the motor in the back right of the bot */
    public static final int BACK_RIGHT_MOTOR = 4;

    /** The high-speed limit for drivetrain movement */
    public static final double HIGH_SPEED = 0.75;

    /** The low-speed limit for drivetrain movement */
    public static final double LOW_SPEED = 0.5;

    /** The width between the wheels across the robot */
    public static final double WHEEL_WIDTH = -1;

    /** The width between the wheels along the robot */
    public static final double WHEEL_LENGTH = -1;
  }

  /** This class holds constants used in tests */
  public static class Tests {
    private Tests() {}

    /** Maximum allowed deviation for tests using doubles */
    public static final double DELTA = 1e-2;
  }
}
