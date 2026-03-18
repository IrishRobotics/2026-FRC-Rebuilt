package infrastructure;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.BeforeEach;

/**
 * Base class for all subsystem tests providing common initialization and cleanup.
 * Ensures HAL is properly initialized, driver station is enabled, and resources are cleaned up.
 */
public abstract class TestBase {

  /**
   * Common setup for all tests. Initializes HAL and enables the driver station.
   * Subclasses should call super.setup() or override this method to add additional setup.
   */
  @BeforeEach
  protected void setup() {
    // Initialize HAL
    assert HAL.initialize(500, 0) : "HAL failed to initialize";
    
    // Enable the driver station simulation
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /**
   * Common cleanup for all tests. Cancels all scheduled commands and disables the driver station.
   * Subclasses should override and call super.cleanup() to add additional cleanup.
   */
  protected void cleanup() {
    // Cancel all scheduled commands
    CommandScheduler.getInstance().cancelAll();
    
    // Disable the driver station simulation
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }
}
