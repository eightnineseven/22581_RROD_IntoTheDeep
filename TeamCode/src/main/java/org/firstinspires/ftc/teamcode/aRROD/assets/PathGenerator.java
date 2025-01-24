package org.firstinspires.ftc.teamcode.aRROD.assets;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
@Config
public class PathGenerator {
    private PathChain preloadTraj, firstSampTraj, firstSampPushTraj, placeSecondSpecTraj, secondSampTraj, placeThirdSpecTraj, thirdSampTraj, placeFourceSpecTraj, getFifthSpecTraj, placeFifthSpecTraj, parkTraj;

    private Follower follower;


    public PathChain getPath(int pathNum, Follower follower) {

        PathBuilder builder = new PathBuilder();
        if (pathNum == 1) {
            builder
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(new Pose(7.338, 65.859, Math.toRadians(0))),
                                    new Point(new Pose(40, 65.859, Math.toRadians(0)))))
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if (pathNum == 2) {
            //break for subsystem
            builder
                    .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(50, 30.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(50, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(22, 20.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 20.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(50, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 13.5, Math.toRadians(0))),
                        new Point(new Pose(22, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 13.5, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 7.9, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(50.000, 7.9, Point.CARTESIAN),
                                new Point(19, 7.9, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
        }
        //break for subsysems
        else if (pathNum == 3) {
            builder
                    .addPath(
                            // Line 6
                            new BezierLine(
                                    new Point(8.686, 24.129, Point.CARTESIAN),
                                    new Point(40.150, 63.507, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==4) {
            builder
                    //break for movement for each one after as well

                    .addPath(
                            // Line 7
                            new BezierLine(
                                    new Point(40.150, 63.507, Point.CARTESIAN),
                                    new Point(8.300, 23.936, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==5) {
               builder
                       .addPath(
                    // Line 8
                    new BezierLine(
                            new Point(8.300, 23.936, Point.CARTESIAN),
                            new Point(39.957, 62.735, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==6) {
               builder
                       .addPath(
                    // Line 9
                    new BezierLine(
                            new Point(39.957, 62.735, Point.CARTESIAN),
                            new Point(8.493, 23.743, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==7) {
               builder
                       .addPath(
                    // Line 10
                    new BezierLine(
                            new Point(8.493, 23.743, Point.CARTESIAN),
                            new Point(39.571, 61.769, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }else if(pathNum==8) {
                builder
                        .addPath(
                    // Line 11
                    new BezierLine(
                            new Point(39.571, 61.769, Point.CARTESIAN),
                            new Point(8.493, 23.357, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }else if(pathNum==9) {
                builder
                        .addPath(
                    // Line 12
                    new BezierLine(
                            new Point(8.493, 23.357, Point.CARTESIAN),
                            new Point(39.764, 60.418, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==10) {
                builder.addPath(
                    // Line 13
                    new BezierLine(
                            new Point(39.764, 60.418, Point.CARTESIAN),
                            new Point(10.231, 37.255, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }
            return builder.build();


        }
    }

