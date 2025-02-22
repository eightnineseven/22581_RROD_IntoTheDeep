package org.firstinspires.ftc.teamcode.aRROD.Auto;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.aRROD.assets.FiveSpecTestPaths;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.commands.commandHolder;

import org.firstinspires.ftc.teamcode.aRROD.commands.FollowPathCommand;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.aRROD.utils.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class fourSpecWithFivePathing extends CommandOpMode {



    public double POWER = 0.9;

    public void initialize() {



        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        Mechanisms mechs = new Mechanisms(hardwareMap, follower);

        follower.setStartingPose(new Pose(7.000, 64.950, 0));

        FiveSpecTestPaths paths = new FiveSpecTestPaths();
        commandHolder actions = new commandHolder();


        schedule(
                //updates our odo every cycle
                new RunCommand(follower::update),
                //updates the double extendo to keep it closed
                new RunCommand(mechs::keepExtendoIn),
                //updates the lift motors so they follow our actions
                new RunCommand(mechs::liftUpdate),
                new FixedSequentialCommandGroup(
                        //initializing movements

                        //waiting to hit play on the driver station
                        new WaitUntilCommand(this::opModeIsActive),
                        actions.prep_score(mechs),
                        //follows first defined path
                        new FollowPathCommand(follower, paths.getPath(1,follower),0.8),


                        //stops this flow until the robot is stopped in place
                        new WaitUntilCommand(() ->follower.getCurrentTValue()>0.89),

                        actions.lift_specimen_score(mechs),
                        //the lift system fires up and down
                        // actions.lift_specimen_score(mechs),



                        new FollowPathCommand(follower, paths.getPath(2,follower),1),
                        actions.pre_wall_intake(mechs),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(700),
                        actions.actual_wall_intake(mechs),




                        new FollowPathCommand(follower, paths.getPath(3,follower),0.8),
                        actions.prep_score(mechs),
                        new WaitUntilCommand(() ->follower.getCurrentTValue()>0.89),
                        actions.lift_specimen_score(mechs),


                        new FollowPathCommand(follower, paths.getPath(4,follower),0.8),
                        actions.pre_wall_intake(mechs),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(700),
                        actions.actual_wall_intake(mechs),



                        new FollowPathCommand(follower, paths.getPath(5,follower),0.8),
                        actions.prep_score(mechs),
                        new WaitUntilCommand(() ->follower.getCurrentTValue()>0.89),
                        actions.lift_specimen_score(mechs),



                        new FollowPathCommand(follower, paths.getPath(6,follower),0.8),
                        actions.pre_wall_intake(mechs),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(700),
                        actions.actual_wall_intake(mechs),



                        new FollowPathCommand(follower, paths.getPath(7,follower),0.8),
                        actions.prep_score(mechs),
                        new WaitUntilCommand(() ->follower.getCurrentTValue()>0.89),
                        actions.lift_specimen_score(mechs),




                        new FollowPathCommand(follower, paths.getPath(10,follower),1),
                        new WaitUntilCommand(() ->!follower.isBusy())

































                )

        );
    }
}
