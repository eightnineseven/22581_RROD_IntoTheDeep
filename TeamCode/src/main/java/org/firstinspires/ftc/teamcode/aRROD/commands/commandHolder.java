package org.firstinspires.ftc.teamcode.aRROD.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;


public class commandHolder {
    public SequentialCommandGroup pre_wall_intake(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::liftDown),
               new InstantCommand(mechs::armIntake),
                new InstantCommand(mechs::clawOpen)



        );
        return  actions;
    }
    public SequentialCommandGroup actual_wall_intake(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::clawClosed),
                new WaitCommand(50),
                new InstantCommand(mechs::armScore),
                new InstantCommand(mechs::liftIntermediate)

        );
        return  actions;
    }


    public SequentialCommandGroup lift_specimen_score(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::lift_specimen),
                new WaitUntilCommand(mechs::liftCloseEnough),
                new InstantCommand(mechs::liftDown),
                new InstantCommand(mechs::clawOpen),
                new InstantCommand(mechs::armIntake)

        );

        return actions;
    }








    public SequentialCommandGroup prep_score(Mechanisms mechs){
        SequentialCommandGroup actions = new SequentialCommandGroup(
                new InstantCommand(mechs::clawClosed),
                new InstantCommand(mechs::armScore),
               new InstantCommand(mechs::liftIntermediate)
        );
        return actions;
    }

}
