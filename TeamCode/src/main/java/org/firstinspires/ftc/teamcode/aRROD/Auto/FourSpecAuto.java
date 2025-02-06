package org.firstinspires.ftc.teamcode.aRROD.Auto;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.assets.FourSpecPaths;
import org.firstinspires.ftc.teamcode.aRROD.assets.commandHolder;

import org.firstinspires.ftc.teamcode.aRROD.commands.FollowPathCommand;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.aRROD.utils.FixedSequentialCommandGroup;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous
public class FourSpecAuto extends CommandOpMode {





    public void initialize() {
        TEAM_COLOR = teamColor.RED;


        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        Mechanisms mechs = new Mechanisms(hardwareMap, follower);

        follower.setStartingPose(new Pose(7.2, 65, 0));

        FourSpecPaths paths = new FourSpecPaths();
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
                        actions.init(mechs),
                        //waiting to hit play on the driver station
                        new WaitUntilCommand(this::opModeIsActive),
                        //follows first defined path
                        new FollowPathCommand(follower, paths.getPath(1,follower)),
                        //stops this flow until the robot is stopped in place
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(200),
                        actions.lift_specimen_score(mechs),
                        actions.pre_wall_intake(mechs),
                        //the lift system fires up and down
                       // actions.lift_specimen_score(mechs),



                        new FollowPathCommand(follower, paths.getPath(2,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new FollowPathCommand(follower, paths.getPath(11,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(800),
                        actions.actual_wall_intake(mechs),



                        new FollowPathCommand(follower, paths.getPath(3,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(200),
                        actions.lift_specimen_score(mechs),
                        actions.pre_wall_intake(mechs),

                        new FollowPathCommand(follower, paths.getPath(4,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(800),
                        actions.actual_wall_intake(mechs),


                        new FollowPathCommand(follower, paths.getPath(5,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(200),
                        actions.lift_specimen_score(mechs),
                        actions.pre_wall_intake(mechs),


                        new FollowPathCommand(follower, paths.getPath(6,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(800),
                        actions.actual_wall_intake(mechs),


                        new FollowPathCommand(follower, paths.getPath(7,follower)),
                        new WaitUntilCommand(() ->!follower.isBusy()),
                        new WaitCommand(200),
                        actions.lift_specimen_score(mechs),
                        actions.pre_wall_intake(mechs),

                new FollowPathCommand(follower, paths.getPath(10,follower)),
                new WaitUntilCommand(() ->!follower.isBusy()),
                        new InstantCommand(()->mechs.autoPosEnd(follower.getPose()))








                        //actions.actual_wall_intake(mechs)




//                        actions.actual_wall_intake(mechs),
//                        new WaitCommand( 300),
//                        new FollowPathCommand(follower, paths.getPath(3,follower)),
//                      new WaitUntilCommand(() ->!follower.isBusy()),
//                        new WaitCommand(1000),
//
//                       actions.lift_specimen_score(mechs),
//                        new FollowPathCommand(follower, paths.getPath(4,follower)),
//                        actions.pre_wall_intake(mechs),
//                        new WaitUntilCommand(() ->!follower.isBusy()),
//                         new WaitCommand(1000),
//                        actions.actual_wall_intake(mechs),
//                        new WaitCommand( 300),
//                        new FollowPathCommand(follower, paths.getPath(5,follower)),
//                        new WaitUntilCommand(() ->!follower.isBusy()),
//                        new WaitCommand(1000),
//                        actions.lift_specimen_score(mechs),
//                        new FollowPathCommand(follower, paths.getPath(6,follower)),
//                        new WaitUntilCommand(mechs::endOfPath)
























                )

        );
    }
}