package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Debug {
  public static Command triggered(String key) {
    SmartDashboard.putBoolean(key, false);
    return Commands.startEnd(
        () -> SmartDashboard.putBoolean(key, true), () -> SmartDashboard.putBoolean(key, false));
  }
}
