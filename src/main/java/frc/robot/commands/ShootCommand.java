package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends ParallelCommandGroup {
    public  ShootCommand(Shooter shooter, Feeder feeder) {
        addCommands(shooter.runAtSpeed(Constants.Shooter.SHOOTER_RPM));
        addCommands(Commands.sequence(new WaitCommand(Constants.Feeder.FEEDER_WAIT),
                new RunFeeder(feeder, Constants.Feeder.FEEDER_POWER)));
    }
}
