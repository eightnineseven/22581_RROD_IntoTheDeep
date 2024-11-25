package org.firstinspires.ftc.teamcode.aRROD.assets;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.aRROD.utils.FixedSequentialCommandGroup;

import java.util.Arrays;

public abstract class AutonomousCommandOpMode extends CommandOpMode {

    /**
     * Schedules the commands to run since INIT is pressed.
     *
     * @param commands The commands to run
     */
    public void scheduleOnInit(Command... commands) {
        schedule(commands);
    }

    /**
     * Schedules the commands to begin once the RUN button is pressed,
     *
     * @param commands The commands to run
     */
    public void scheduleOnRun(Command... commands) {
        schedule(Arrays.stream(commands)
                .map(command -> new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        command
                ))
                .toArray(Command[]::new));
    }

    @Override
    public void runOpMode() {
        try {
            initialize();

            while (opModeInInit() || opModeIsActive()) {
                run();
            }

            if (isStopRequested()) // TODO: Check if this is safe
                CommandScheduler.getInstance().cancelAll();

        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            reset();
        }
    }
}
