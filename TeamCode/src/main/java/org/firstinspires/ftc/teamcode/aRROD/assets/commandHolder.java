package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.SERVO_WAIT_TIME;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import com.pedropathing.follower.Follower;


public class commandHolder {
    public SequentialCommandGroup pre_wall_intake(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
               new InstantCommand(mechs::armIntake),
                new InstantCommand(mechs::clawOpen)



        );
        return  actions;
    }
    public SequentialCommandGroup actual_wall_intake(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::clawClosed),
                new WaitCommand(200),
                new InstantCommand(mechs::armScore)

        );
        return  actions;
    }


    public SequentialCommandGroup lift_specimen_score(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::liftUp),
                new WaitUntilCommand(mechs::liftCloseEnough),
                new WaitCommand(300),
                new InstantCommand(mechs::clawOpen),
                new InstantCommand(mechs::liftDown)
        );

        return actions;
    }








    public SequentialCommandGroup init(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::clawClosed),
                new InstantCommand(mechs::armScore),
                new InstantCommand(mechs::extendoClose)
        );
        return actions;
    }

}
