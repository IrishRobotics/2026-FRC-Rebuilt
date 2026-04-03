package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class LessDumbAuto extends SequentialCommandGroup {
    public LessDumbAuto(Drivetrain drivetrain, Shooter shooter, Feeder feeder) {
        addCommands(new MoveFront(drivetrain, 2), new ShootCommand(shooter, feeder).withDeadline(new WaitCommand(8)));
    }
}
