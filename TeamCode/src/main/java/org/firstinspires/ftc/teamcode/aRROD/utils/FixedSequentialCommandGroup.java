package org.firstinspires.ftc.teamcode.aRROD.utils;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class FixedSequentialCommandGroup extends SequentialCommandGroup {
    private boolean finished;

    public FixedSequentialCommandGroup(Command... commands) {
        super(commands);
    }

    @Override
    public void initialize() {
        super.initialize();
        finished = false;
    }

    @Override
    public void execute() {
        if (finished)
            return;

        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        if (finished)
            return;

        super.end(interrupted);
        finished = true;
    }

    @Override
    public boolean isFinished() {
        finished |= super.isFinished();
        return finished;
    }
}
