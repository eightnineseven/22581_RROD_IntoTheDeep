package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.SERVO_WAIT_TIME;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.follower.Follower;


public class commandHolder {
    public SequentialCommandGroup lift_arm_specimen_intake(Mechanisms mechs, Follower follower){
        SequentialCommandGroup actions = new SequentialCommandGroup(
               new InstantCommand(mechs::armIntake),
                new InstantCommand(mechs::clawOpen),
                new WaitUntilCommand(follower::isBusy),
                new InstantCommand(mechs::clawClosed),
                new WaitCommand(SERVO_WAIT_TIME),
                new InstantCommand(mechs::armScore)

        );
        return  actions;
    }


    public SequentialCommandGroup lift_specimen_score(Mechanisms mechs, Follower follower){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::liftUp),
                new WaitUntilCommand(mechs::liftCloseEnough),
                new InstantCommand(mechs::liftDown)
        );
        return actions;
    }



    public SequentialCommandGroup init(Mechanisms mechs, Follower follower){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::clawClosed),
                new WaitCommand(300),
                new InstantCommand(mechs::armScore),
                new InstantCommand(mechs::extendoClose)
        );
        return actions;
    }

}
