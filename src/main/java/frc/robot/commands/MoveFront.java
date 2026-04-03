package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class MoveFront extends Command {
    private Drivetrain drivetrain;
    private double distance;

    public MoveFront(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        addRequirements(drivetrain);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0);
    }

    @Override
    public void execute() {
        drivetrain.drive(0.4, 0, 0);
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.sqrt(Math.pow(drivetrain.getPose().getX(), 2) + Math.pow(drivetrain.getPose().getY(), 2)) >= distance;
    }
}
