// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Debug {
  /**
   * Creates a Command that exposes a transient boolean flag on the
   * SmartDashboard.
   *
   * The SmartDashboard entry identified by {@code key} is initialized to
   * {@code false}
   * at creation time. When the returned command is started it sets the flag to
   * {@code true},
   * and when the command ends (or is interrupted) it sets the flag back to
   * {@code false}.
   *
   * This is intended for temporary debug/telemetry indicators that should reflect
   * whether
   * a command is currently active.
   *
   * @param key the SmartDashboard key to use for the boolean flag; must not be
   *            {@code null}
   * @return a Command which sets the SmartDashboard boolean to {@code true} on
   *         start and
   *         to {@code false} on end
   */
  public static Command triggered(String key) {
    SmartDashboard.putBoolean(key, false);
    return Commands.startEnd(
        () -> SmartDashboard.putBoolean(key, true),
        () -> SmartDashboard.putBoolean(key, false));
  }
}
