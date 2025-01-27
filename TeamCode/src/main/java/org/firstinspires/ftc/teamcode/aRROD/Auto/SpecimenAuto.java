package org.firstinspires.ftc.teamcode.aRROD.Auto;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.assets.PathGenerator;
import org.firstinspires.ftc.teamcode.aRROD.assets.commandHolder;

import org.firstinspires.ftc.teamcode.aRROD.commands.FollowPathCommand;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.aRROD.utils.FixedSequentialCommandGroup;

@Config
@Autonomous
public class SpecimenAuto extends CommandOpMode {





    public void initialize() {
        TEAM_COLOR = teamColor.RED;

        Mechanisms mechs = new Mechanisms(hardwareMap);

        Follower follower = new Follower(hardwareMap);

        //follower.setStartingPose(new Pose(startX, startY, 180));

        PathGenerator paths = new PathGenerator();
        commandHolder actions = new commandHolder();


        schedule(
                //updates our odo every cycle
                new RunCommand(follower::update),
                //updates the double extendo to keep it closed
                new RunCommand(mechs::extendoUpdate),
                //updates the lift motors so they follow our actions
                new RunCommand(mechs::liftUpdate),
                new FixedSequentialCommandGroup(
                        //initializing movements
                        actions.init(mechs, follower),
                        //waiting to hit play on the driver station
                        new WaitUntilCommand(this::opModeIsActive),
                        //follows first defined path
                        new FollowPathCommand(follower, paths.getPath(1,follower)),
                        //stops this flow until the robot is stopped in place
                        new WaitUntilCommand(follower::isBusy),
                        //the lift system fires up and down
                        actions.lift_specimen_score(mechs,follower),
                        new FollowPathCommand(follower, paths.getPath(2,follower)),
                        actions.lift_arm_specimen_intake(mechs, follower),
                        new FollowPathCommand(follower, paths.getPath(3,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        actions.lift_specimen_score(mechs,follower),
                        new FollowPathCommand(follower, paths.getPath(4,follower)),
                        actions.lift_arm_specimen_intake(mechs, follower),
                        new FollowPathCommand(follower, paths.getPath(5,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        actions.lift_specimen_score(mechs,follower),
                        new FollowPathCommand(follower, paths.getPath(6,follower)),
                        actions.lift_arm_specimen_intake(mechs, follower),
                        new FollowPathCommand(follower, paths.getPath(7,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        actions.lift_specimen_score(mechs,follower),
                        new FollowPathCommand(follower, paths.getPath(8,follower)),
                        actions.lift_arm_specimen_intake(mechs, follower),
                        new FollowPathCommand(follower, paths.getPath(9,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        actions.lift_specimen_score(mechs,follower),
                        new FollowPathCommand(follower, paths.getPath(10,follower)),
                        new WaitUntilCommand(follower::isBusy)





















                        )




        );
    }
}