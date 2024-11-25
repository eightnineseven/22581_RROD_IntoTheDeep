package org.firstinspires.ftc.teamcode.aRROD.Auto;

import static org.firstinspires.ftc.teamcode.aRROD.Auto.Blue_1_plus_3.HEADING_SUB;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.teamcode.aRROD.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.aRROD.assets.Arm;
import org.firstinspires.ftc.teamcode.aRROD.assets.IntakeServos;
import org.firstinspires.ftc.teamcode.aRROD.assets.PathGenerator;
import org.firstinspires.ftc.teamcode.aRROD.commands.FollowPathCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.aRROD.utils.FixedSequentialCommandGroup;

@Config
@Autonomous
public class redWCommandBase extends CommandOpMode {


    private final Point stackPoint = new Point(144-34, 144-14, Point.CARTESIAN);
    private final Point backdropPoint = new Point(144-29, 144-120, Point.CARTESIAN);


    public void initialize() {
        TEAM_COLOR = teamColor.RED;
        IntakeServos intake = new IntakeServos(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        Follower follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose(startX, startY, HEADING_SUB));

        PathGenerator paths = new PathGenerator();

//        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
//        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);
//        RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

//        PropLocation propLocation = PropLocation.TRUSS_SIDE;
//        Path purplePath = propLocation.getPurplePath(allianceColor);
//        Path yellowPath = propLocation.getYellowPath(allianceColor);

//        PathChain firstStackPath = stackPaths.getStackPath(yellowPath.getLastControlPoint(), StackGenerator.Route.WALL_TRUSS);
//        PathChain secondStackPath = stackPaths.getStackPath(backdropPoint, StackGenerator.Route.WALL_TRUSS);
//        PathChain backdropPath = stackPaths.getBackdropPath(StackGenerator.Route.WALL_TRUSS);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(arm::armUpdate),
                new FixedSequentialCommandGroup(
                        new InstantCommand(intake::closeClaw)
                                .alongWith(
                                        new InstantCommand(intake::flipIn)
                                ),

                        new WaitUntilCommand(this::opModeIsActive),

                        // Start purple following

                        new FollowPathCommand(follower, paths.getPath(1, follower)),
                        new WaitUntilCommand(follower::isBusy),



                        new InstantCommand(arm::outtake),
                        new WaitCommand(700),
                        new FollowPathCommand(follower,paths.getPath(10,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        new InstantCommand(intake::flipOut)
                                .andThen(
                                        new WaitCommand(600),
                                        new InstantCommand(arm::intermediate)
                                ),
                        new WaitCommand(400),
                        new InstantCommand(intake::openClaw),
                        new InstantCommand(intake::flipIn),
                        new FollowPathCommand(follower, paths.getPath(11,follower)),
                        new WaitUntilCommand(follower::isBusy),
                        new InstantCommand(arm::rest),







                        new FollowPathCommand(follower, paths.getPath(2, follower)),
                        new WaitCommand(4000),
                        //arm up



                        new InstantCommand(intake::openClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipOut)
                                ),
                        new InstantCommand(arm::intake),

                        new WaitCommand(1000),
                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1500),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new WaitCommand(400),
                        new InstantCommand(arm::rest),



                        new FollowPathCommand(follower, paths.getPath(4,follower)),
                        new WaitUntilCommand(follower::isBusy)
                                .alongWith(
                                        new WaitCommand(2000),
                                        new InstantCommand(arm::resetArm)
                                ),



                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new InstantCommand(arm::outtake),
                        new WaitCommand(1500),
                        new InstantCommand(intake::flipOut)
                                .andThen(
                                        new WaitCommand(600),
                                        new InstantCommand(arm::rest)
                                ),
                        new WaitCommand(400),
                        new InstantCommand(intake::openClaw),
                        new InstantCommand(intake::flipIn),






                        new FollowPathCommand(follower, paths.getPath(5,follower)),
                        new WaitCommand(4000),

                        new InstantCommand(intake::openClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipOut)
                                ),
                        new InstantCommand(arm::intake),
                        new WaitCommand(1000),
                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1500),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new WaitCommand(400),

                        new InstantCommand(arm::rest),








                        new FollowPathCommand(follower, paths.getPath(6,follower)),
                        new WaitUntilCommand(follower::isBusy)
                                .alongWith(
                                        new WaitCommand(2000),
                                        new InstantCommand(arm::resetArm)
                                ),

                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new InstantCommand(arm::outtake),
                        new WaitCommand(1500),
                        new InstantCommand(intake::flipOut)
                                .andThen(
                                        new WaitCommand(600),
                                        new InstantCommand(arm::rest)
                                ),
                        new WaitCommand(400),
                        new InstantCommand(intake::openClaw),
                        new InstantCommand(intake::flipIn),





                        new FollowPathCommand(follower, paths.getPath(7,follower)),
                        new WaitUntilCommand(follower::isBusy)
                                .alongWith(
                                        new WaitCommand(1000),
                                        new InstantCommand(arm::resetArm)
                                ),



                        new InstantCommand(intake::openClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipOut)
                                ),
                        new InstantCommand(arm::intake),
                        new WaitCommand(1000),
                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1500),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new WaitCommand(400),

                        new InstantCommand(arm::rest),







                        new FollowPathCommand(follower, paths.getPath(8,follower)),
                        new WaitUntilCommand(follower::isBusy)
                                .alongWith(
                                        new WaitCommand(1000),
                                        new InstantCommand(arm::resetArm)
                                ),



                        new InstantCommand(intake::closeClaw)
                                .andThen(
                                        new WaitCommand(1000),
                                        new InstantCommand(intake::flipIn)
                                ),
                        new InstantCommand(arm::outtake),
                        new WaitCommand(1500),
                        new InstantCommand(intake::flipOut)
                                .andThen(
                                        new WaitCommand(600),
                                        new InstantCommand(arm::rest)
                                ),
                        new WaitCommand(400),
                        new InstantCommand(intake::openClaw),
                        new InstantCommand(intake::flipIn),




                        new FollowPathCommand(follower, paths.getPath(9,follower)),
                        new WaitUntilCommand(follower::isBusy)









                )




        );
    }
}