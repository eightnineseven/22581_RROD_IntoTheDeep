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
public class FiveSpecPaths {
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
                                    new Point(42.000, 78.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));

        } else if (pathNum == 2) {
            //break for subsystem
            builder

                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(42.000, 78.000, Point.CARTESIAN),
                                    new Point(19.619, 20.612, Point.CARTESIAN),
                                    new Point(40.373, 38.381, Point.CARTESIAN),
                                    new Point(76.000, 48.000, Point.CARTESIAN),
                                    new Point(81.000, 16.000, Point.CARTESIAN),
                                    new Point(45.000, 24.000, Point.CARTESIAN),
                                    new Point(20.000, 24.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierCurve(
                                    new Point(20.000, 24.000, Point.CARTESIAN),
                                    new Point(107.000, 26.000, Point.CARTESIAN),
                                    new Point(45.000, 10.000, Point.CARTESIAN),
                                    new Point(27.000, 17.000, Point.CARTESIAN),
                                    new Point(20.000, 16.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierCurve(
                                    new Point(20.000, 16.000, Point.CARTESIAN),
                                    new Point(94.000, 17.000, Point.CARTESIAN),
                                    new Point(82.000, 5.000, Point.CARTESIAN),
                                    new Point(48.000, 9.000, Point.CARTESIAN),
                                    new Point(11.000, 5.000, Point.CARTESIAN),
                                    new Point(13.000, 4.000, Point.CARTESIAN),
                                    new Point(7.600, 32.000, Point.CARTESIAN)
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
                            //place fisrt
                            new BezierLine(
                                    new Point(7.600, 32.000, Point.CARTESIAN),
                                    new Point(42.000, 75.000, Point.CARTESIAN)
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
                                    new Point(42.000, 75.000, Point.CARTESIAN),
                                    new Point(7.600, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==5) {
            builder
                    .addPath(
                            // Line 8
                            //place second
                            new BezierLine(
                                    new Point(7.600, 32.000, Point.CARTESIAN),
                                    new Point(42.000, 72.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==6) {
            builder
                    .addPath(
                            // Line 9
                            //pickup 3rd
                            new BezierLine(
                                    new Point(42.000, 72.000, Point.CARTESIAN),
                                    new Point(7.600, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==7){
            builder
                    .addPath(
                            // Line 9
                            //place third
                            new BezierLine(
                                    new Point(7.600, 32.000, Point.CARTESIAN),
                                    new Point(42.000, 69.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==8){
            builder
                    .addPath(
                            // Line 9
                            //pickup 4th
                            new BezierLine(
                                    new Point(42.000, 69.000, Point.CARTESIAN),
                                    new Point(7.600, 32.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==9){
            builder
                    .addPath(
                            // Line 9
                            //place 4th
                            new BezierLine(
                                    new Point(7.600, 32.000, Point.CARTESIAN),
                                    new Point(42.000, 66.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        } else if(pathNum==10){
            builder
                    .addPath(
                            // Line 9
                            //park
                            new BezierLine(
                                    new Point(42.000, 66.000, Point.CARTESIAN),
                                    new Point(7.500, 40.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));
        }

        return builder.build();


    }
}
