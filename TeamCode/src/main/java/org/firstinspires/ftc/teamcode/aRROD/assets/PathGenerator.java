package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.Auto.Blue_1_plus_3.HEADING_SUB;
import static org.firstinspires.ftc.teamcode.aRROD.Auto.Blue_1_plus_3.HEADING_WALL;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
@Config
public class PathGenerator {
    private PathChain preloadTraj, firstSampTraj, firstSampPushTraj, placeSecondSpecTraj, secondSampTraj, placeThirdSpecTraj, thirdSampTraj, placeFourceSpecTraj, getFifthSpecTraj, placeFifthSpecTraj, parkTraj;

    private Follower follower;


    public PathChain getPath(int pathNum, Follower follower) {

        PathBuilder builder = new PathBuilder();
        if (TEAM_COLOR.equals(teamColor.BLUE)) {
            if (pathNum == 1) {
                builder
                        .addBezierLine(new Point(startX, startY), new Point(preloadPlaceX, preloadPlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)

                        .build();
            } else if (pathNum == 2) {
                builder
                        .addBezierCurve(new Point(sample1X, sample1Y, Point.CARTESIAN), new Point(firstSample1WeightX, firstSample1WeightY, Point.CARTESIAN), new Point(secondSample1WeightX, secondSample1WeightY, Point.CARTESIAN))
                        .addBezierCurve(new Point(sample1X, sample1Y, Point.CARTESIAN), new Point(sample1X, sample1Y, Point.CARTESIAN))

                        .setConstantHeadingInterpolation(HEADING_SUB)

                        .build();


            } else if (pathNum == 4) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeThirdSpecX, placeThirdSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)

                        .build();
            } else if (pathNum == 5) {
                builder
                        .addBezierCurve(new Point(sample2X, sample2Y, Point.CARTESIAN), new Point(firstSample2WeightX, firstSample2WeightY, Point.CARTESIAN), new Point(secondSample2WeightX, secondSample2WeightY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_WALL)
                        .setPathEndHeadingConstraint(0.07)
                        .addBezierCurve(new Point(sample2X, sample2Y, Point.CARTESIAN), new Point(sample2X, sample2Y, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_WALL)
                        .setPathEndHeadingConstraint(0.07)


                        .build();
            } else if (pathNum == 6) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeThirdSpecX, placeThirdSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)


                        .build();
            } else if (pathNum == 7) {
                builder
                        .addBezierCurve(new Point(sample3X, sample3Y, Point.CARTESIAN), new Point(sample3X, sample3Y, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_WALL)


                        .build();
            } else if (pathNum == 8) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeFourthSpecX, placeFourthSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)


                        .build();
            } else if (pathNum == 9) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(parkX, parkY, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .setPathEndTValueConstraint(0.01)

                        .build();
            } else if (pathNum == 10) {
                builder
                        .addBezierLine(new Point(follower.getPose()), new Point(prePlaceX, prePlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .build();
            } else if (pathNum == 11) {
                builder
                        .addBezierLine(new Point(follower.getPose()), new Point(postPlaceX, postPlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .build();
            }
            return builder.build();


        } else {
            if (pathNum == 1) {
                builder
                        .addBezierLine(new Point(144-startX, 144-startY), new Point(144-preloadPlaceX, 144-preloadPlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)

                        .build();
            } else if (pathNum == 2) {
                builder
                        .addBezierCurve(new Point(144-sample1X, 144-sample1Y, Point.CARTESIAN), new Point(144-firstSample1WeightX, 144-firstSample1WeightY, Point.CARTESIAN), new Point(144-secondSample1WeightX, 144-secondSample1WeightY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_WALL)
                        .addBezierCurve(new Point(144-sample1X, 144-sample1Y, Point.CARTESIAN), new Point(144-sample1X, 144-sample1Y, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_WALL)


                        .build();


            } else if (pathNum == 4) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeSecondSpecX, 144-placeSecondSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)


                        .build();
            } else if (pathNum == 5) {
                builder
                        .addBezierCurve(new Point(144-sample2X, 144-sample2Y, Point.CARTESIAN), new Point(144-firstSample2WeightX, 144-firstSample2WeightY, Point.CARTESIAN), new Point(144-secondSample2WeightX, 144-secondSample2WeightY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_WALL)
                        .setPathEndHeadingConstraint(0.07)
                        .addBezierCurve(new Point(144-sample2X, 144-sample2Y, Point.CARTESIAN), new Point(144-sample2X, 144-sample2Y, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_WALL)
                        .setPathEndHeadingConstraint(0.07)


                        .build();
            } else if (pathNum == 6) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeThirdSpecX, 144-placeThirdSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)


                        .build();
            } else if (pathNum == 7) {
                builder
                        .addBezierCurve(new Point(144-sample3X, 144-sample3Y, Point.CARTESIAN), new Point(144-sample3X, 144-sample3Y, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_WALL)


                        .build();
            } else if (pathNum == 8) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeFourthSpecX, 144-placeFourthSpecY, Point.CARTESIAN))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)


                        .build();
            } else if (pathNum == 9) {
                builder
                        .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-parkX, 144-parkY, Point.CARTESIAN))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .setPathEndTValueConstraint(0.01)

                        .build();
            } else if (pathNum == 10) {
                builder
                        .addBezierLine(new Point(follower.getPose()), new Point(144-prePlaceX, 144-prePlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .build();
            } else if (pathNum == 11) {
                builder
                        .addBezierLine(new Point(follower.getPose()), new Point(144-postPlaceX, 144-postPlaceY))
                        .setConstantHeadingInterpolation(HEADING_SUB)
                        .build();
            }
            return builder.build();


        }
    }
}
