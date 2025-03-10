package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPooper;

public class CoralColon extends Command {
    private final CoralPooper outtake;
    private final double speed;

    public CoralColon(CoralPooper outtake, double speed) {
        this.outtake = outtake;
        this.speed = speed;
        addRequirements(outtake);
    }

    @Override
    public void execute() {
        outtake.setColonSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setColonSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
