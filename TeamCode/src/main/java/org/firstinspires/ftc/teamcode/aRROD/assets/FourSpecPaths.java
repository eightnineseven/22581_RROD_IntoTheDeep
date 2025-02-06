package org.firstinspires.ftc.teamcode.aRROD.assets;


import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
public class FourSpecPaths {
    private PathChain preloadTraj, firstSampTraj, firstSampPushTraj, placeSecondSpecTraj, secondSampTraj, placeThirdSpecTraj, thirdSampTraj, placeFourceSpecTraj, getFifthSpecTraj, placeFifthSpecTraj, parkTraj;

    private PinpointLocalizer follower;


    public PathChain getPath(int pathNum, Follower follower) {

        PathBuilder builder = new PathBuilder();
        if (pathNum == 1) {
            builder
                    .addPath(
                            // Line 1
                    //preload
                            new BezierLine(
                                    new Point(7.528, 65.437, Point.CARTESIAN),
                                    new Point(33.3, 78, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));

        } else if (pathNum == 2) {
            //break for subsystem
            builder
                    //before first
                    .addPath(  new BezierCurve(
                                    new Point(33.3, 78, Point.CARTESIAN),
                            new Point(16.214, 26.252, Point.CARTESIAN),
                            new Point(48.60, 39.131, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                    //first push
                            // Line 3
                            new BezierCurve(
                                    new Point(48.60, 39.131, Point.CARTESIAN),
                                    new Point(78.212, 7.601, Point.CARTESIAN),
                                    new Point(20.000, 32.287, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                    //before second
                            new BezierLine(
                                    new Point(20.000, 32.287, Point.CARTESIAN),
                                    new Point(46.365, 24.198, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 5
                            //second push
                            new BezierCurve(
                                    new Point(46.365, 24.198, Point.CARTESIAN),
                                    new Point(70.842, 10.686, Point.CARTESIAN),
                                    new Point(28.000, 17.898, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));




        } else if (pathNum == 3) {
            builder
                            // Line 6
                    .addPath(
                            // Line 8
                            //place fisrt
                            new BezierLine(
                                    new Point(6.8, 27, Point.CARTESIAN),

                                    new Point(33.7, 74, Point.CARTESIAN)
                            )


                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==4) {
            builder
                    //break for movement for each one after as well
                    //pickup second
                    .addPath(
                            // Line 7
                            new BezierLine(
                                    new Point(33.7, 74, Point.CARTESIAN),
                                    new Point(6.8, 27, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==5) {
               builder
                       .addPath(
                    // Line 8
                               //place second
                               new BezierLine(
                                       new Point(6.8, 27, Point.CARTESIAN),
                                       new Point(33.5, 70, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==6) {
               builder
                       .addPath(
                    // Line 9
                     //pickup 3rd
                    new BezierLine(
                                       new Point(33.5, 70, Point.CARTESIAN),
                                       new Point(6.8, 27, Point.CARTESIAN)
                    )
            )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==7){
            builder
                    .addPath(
                            // Line 9
                            //place third
                            new BezierLine(
                                    new Point(6.8, 27, Point.CARTESIAN),
                                    new Point(33, 67, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==8){
            builder
                    .addPath(
                            // Line 9
                            //pickup 4th
                            new BezierLine(
                                    new Point(33, 67, Point.CARTESIAN),
                                    new Point(6.8, 27, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==9){
            builder
                    .addPath(
                            // Line 9
                            //place 4th
                            new BezierLine(
                                    new Point(6.8, 27, Point.CARTESIAN),
                                    new Point(32.3, 64, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==10){
            builder
                    .addPath(
                            // Line 9
                            //park
                            new BezierLine(
                                    new Point(32.3, 67, Point.CARTESIAN),
                                    new Point(8, 38, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }
        else if(pathNum==11){
            builder
                    .addPath(
                            // Line 9
                            //move to hp
                            new BezierLine(
                                    new Point(28.000, 17.898, Point.CARTESIAN),
                                    new Point(6.8, 27, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }
            return builder.build();


        }
    }

