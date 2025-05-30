package org.frc2910.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandWrapper extends Command {
    private final Command inner;

    public CommandWrapper(Command inner) {
        this.inner = inner;
    }

    @Override
    public void initialize() {
        inner.initialize();
    }

    @Override
    public void execute() {
        inner.execute();
    }

    @Override
    public void end(boolean interrupted) {
        inner.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return inner.isFinished();
    }
}
