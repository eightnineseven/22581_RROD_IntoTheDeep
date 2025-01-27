package org.firstinspires.ftc.teamcode.aRROD.assets;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;


import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

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
                                    new Point(8.493, 65.244, Point.CARTESIAN),
                                    new Point(40.150, 64.472, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if (pathNum == 2) {
            //break for subsystem
            builder
                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(40.150, 64.472, Point.CARTESIAN),
                                    new Point(0.965, 36.869, Point.CARTESIAN),
                                    new Point(71.807, 33.587, Point.CARTESIAN),
                                    new Point(101.534, 18.145, Point.CARTESIAN),
                                    new Point(32.622, 24.901, Point.CARTESIAN),
                                    new Point(20.461, 22.391, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierCurve(
                                    new Point(20.461, 22.391, Point.CARTESIAN),
                                    new Point(65.630, 28.954, Point.CARTESIAN),
                                    new Point(73.544, 8.493, Point.CARTESIAN),
                                    new Point(68.912, 16.214, Point.CARTESIAN),
                                    new Point(20.461, 12.161, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierCurve(
                                    new Point(20.461, 12.161, Point.CARTESIAN),
                                    new Point(84.933, 18.531, Point.CARTESIAN),
                                    new Point(69.298, 4.054, Point.CARTESIAN),
                                    new Point(21.812, 7.914, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 5
                            new BezierLine(
                                    new Point(21.812, 7.914, Point.CARTESIAN),
                                    new Point(8.686, 24.129, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
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

