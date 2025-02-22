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
public class FiveSpecTestPaths {
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
                                    new Point(7.000, 64.950, Point.CARTESIAN),
                                    new Point(41.000, 80.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if (pathNum == 2) {
            //break for subsystem
            builder

                    .addPath(
                            // push 1
                            new BezierCurve(
                                    new Point(41.000, 80.000, Point.CARTESIAN),
                                    new Point(21.426, 20.847, Point.CARTESIAN),
                                    new Point(41.308, 37.641, Point.CARTESIAN),
                                    new Point(76.000, 48.000, Point.CARTESIAN),
                                    new Point(75.000, 16.000, Point.CARTESIAN),
                                    new Point(45.000, 24.000, Point.CARTESIAN),
                                    new Point(25.000, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // push 2
                            new BezierCurve(
                                    new Point(25.000, 24.000, Point.CARTESIAN),
                                    new Point(75.000, 30.000, Point.CARTESIAN),
                                    new Point(47.000, 10.000, Point.CARTESIAN),
                                    new Point(27.000, 17.000, Point.CARTESIAN),
                                    new Point(25.000, 16.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // push 3/pickup 1st
                            new BezierCurve(
                                    new Point(25.000, 16.000, Point.CARTESIAN),
                                    new Point(68.000, 17.000, Point.CARTESIAN),
                                    new Point(79.000, 9.000, Point.CARTESIAN),
                                    new Point(42.000, 9.000, Point.CARTESIAN),
                                    new Point(11.000, 9.000, Point.CARTESIAN),
                                    new Point(13.000, 9, Point.CARTESIAN),
                                    new Point(6.900, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));



        }
        //break for subsysems
        else if (pathNum == 3) {
            builder
                    // Line 6
                    .addPath(
                            // Line 8
                            //place 2nd
                            new BezierCurve(
                                    new Point(7.400, 32.000, Point.CARTESIAN),
                                    new Point(15.000, 74.000, Point.CARTESIAN),
                                    new Point(41.000, 77.500, Point.CARTESIAN)
                            )


                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==4) {
            builder
                    //pickup second
                    .addPath(
                            // Line 7
                            new BezierCurve(
                                    new Point(41.000, 77.500, Point.CARTESIAN),
                                    new Point(17.000, 73.000, Point.CARTESIAN),
                                    new Point(35.000, 27.000, Point.CARTESIAN),
                                    new Point(7.400, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==5) {
            builder
                    .addPath(
                            //place third
                            new BezierCurve(
                                    new Point(7.400, 32.000, Point.CARTESIAN),
                                    new Point(15.000, 74.000, Point.CARTESIAN),
                                    new Point(41.000, 75.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==6) {
            builder
                    .addPath(
                            //pickup 3rd
                            new BezierCurve(
                                    new Point(41.000, 75.000, Point.CARTESIAN),
                                    new Point(17.000, 73.000, Point.CARTESIAN),
                                    new Point(35.000, 27.000, Point.CARTESIAN),
                                    new Point(7.400, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==7){
            builder
                    .addPath(
                            //place fourth
                            new BezierCurve(
                                    new Point(7.400, 32.000, Point.CARTESIAN),
                                    new Point(15.000, 74.000, Point.CARTESIAN),
                                    new Point(41.000, 72.500, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==8){
            builder
                    .addPath(
                            // Line 9
                            //pickup 4th
                            new BezierCurve(
                                    new Point(41.000, 72.500, Point.CARTESIAN),
                                    new Point(17.000, 73.000, Point.CARTESIAN),
                                    new Point(35.000, 27.000, Point.CARTESIAN),
                                    new Point(7.40, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==9){
            builder
                    .addPath(
                            // Line 9
                            //place 5th
                            new BezierCurve(
                                    new Point(7.400, 32.000, Point.CARTESIAN),
                                    new Point(15.000, 61.000, Point.CARTESIAN),
                                    new Point(41.000, 70.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==10){
            builder
                    .addPath(
                            // Line 9
                            //park
                            new BezierLine(
                                    new Point(41.000, 70.000, Point.CARTESIAN),
                                    new Point(7.500, 40.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }

        return builder.build();


    }
}
