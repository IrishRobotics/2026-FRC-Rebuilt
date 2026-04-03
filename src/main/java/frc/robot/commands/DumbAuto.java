package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import java.util.Timer;
import java.util.TimerTask;

public class DumbAuto extends Command {
  private final Drivetrain drivetrain;
  private final Timer timer = new Timer(true);

  public DumbAuto(Drivetrain drive) {
    this.drivetrain = drive;
    addRequirements(drive);
  }

  @Override
  public void end(boolean interrupted) {
    timer.cancel();
    drivetrain.drive(0, 0, 0);
  }

  @Override
  public void execute() {}

  private class StopTurn extends TimerTask {
    @Override
    public void run() {
      drivetrain.drive(0, 0, 0.3);
    }
  }

  @Override
  public void initialize() {
    drivetrain.drive(0.5, 0, 0);
    timer.schedule(new StopTurn(), 1000);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
