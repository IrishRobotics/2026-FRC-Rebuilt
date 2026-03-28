package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

import java.util.Timer;
import java.util.TimerTask;

public class RunFeeder extends Command {
    private final Feeder feeder;
    private final double speed;
    private Timer timer = new Timer(true);
    private boolean started = false;

    public RunFeeder(Feeder feeder, double speed) {
        this.feeder = feeder;
        this.speed = speed;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setSpeed(speed);
        timer.purge();
        started = false;
        timer.schedule(new Started(), 1000);
    }

    @Override
    public void execute() {
        if(feeder.getSpeed() < 60 && started) {
            feeder.setSpeed(-1);
            timer.schedule(new RunForward(), 1000);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.purge();
        feeder.stop();
    }

    private class Started extends TimerTask {
        @Override
        public void run() {
            started = true;
        }
    }

    private class RunForward extends TimerTask {
        @Override
        public void run() {
            feeder.setSpeed(speed);
        }
    }
}