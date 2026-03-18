package general;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base class for all subsystem tests providing common initialization and cleanup. Ensures HAL is
 * properly initialized, driver station is enabled, and resources are cleaned up.
 */
public abstract class TestBase {

  /**
   * Common setup for all tests. Initializes HAL and enables the driver station. Subclasses should
   * call super.setup() or override this method to add additional setup.
   */
  @BeforeEach
  protected void setup() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /**
   * Common cleanup for all tests. Cancels all scheduled commands and disables the driver station.
   * Subclasses should override and call super.cleanup() to add additional cleanup.
   */
  @AfterEach
  protected void cleanup() throws Exception {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }
}
