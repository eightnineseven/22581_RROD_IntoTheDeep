package org.firstinspires.ftc.teamcode.aRROD.Auto;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="RED_1_plus_3",group="aRROD")
public class Red_1_plus_3 extends OpMode {



    public Follower follower;
    private int pathState;
    private int nextCase;
    public Timer timer;
    public boolean letTimerRun = false;
    public double WAIT_TIME=2.7;
    public double SAMP_TIME = 5;
    public static double HEADING_WALL = 0;
    public static double HEADING_SUB = Math.toRadians(180);
    private final int INTAKE = -2;
    private final int OUTTAKE = -1;
    private final int REST = -3;
    public boolean pathJustSwitched = false;
    public boolean autoFinished = false;
    public boolean updateSubsystems = false;
    public int nextRobotAction;

    private PathChain preloadTraj, firstSampTraj, firstSampPushTraj, placeSecondSpecTraj, secondSampTraj, placeThirdSpecTraj, thirdSampTraj, placeFourceSpecTraj, getFifthSpecTraj, placeFifthSpecTraj, parkTraj;

    public void pathBuilder(){
        preloadTraj = follower.pathBuilder()
                .addBezierLine(new Point(144-startX, 144-startY), new Point(144-preloadPlaceX, 144-preloadPlaceY))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        firstSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(144-sample1X, 144-sample1Y, Point.CARTESIAN), new Point(144-firstSample1WeightX, 144-firstSample1WeightY, Point.CARTESIAN), new Point(144-secondSample1WeightX, 144-secondSample1WeightY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .addBezierCurve(new Point(144-sample1X,144-sample1Y,Point.CARTESIAN), new Point(144-sample1X,144-sample1Y,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        firstSampPushTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),new Point(144-7,144-7))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeSecondSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),new Point(144-placeSecondSpecX,144-placeSecondSpecY,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        secondSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(144-sample2X, 144-sample2Y, Point.CARTESIAN), new Point(144-firstSample2WeightX, 144-firstSample2WeightY, Point.CARTESIAN), new Point(144-secondSample2WeightX, 144-secondSample2WeightY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .setPathEndHeadingConstraint(0.07)
                .addBezierCurve(new Point(144-sample2X,144-sample2Y,Point.CARTESIAN),new Point(144-sample2X,144-sample2Y,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .setPathEndHeadingConstraint(0.07)
                .build();
        placeThirdSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeThirdSpecX, 144-placeThirdSpecY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();

        thirdSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(144-sample3X,144-sample3Y,Point.CARTESIAN),new Point(144-sample3X,144-sample3Y,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeFourceSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeFourthSpecX, 144-placeFourthSpecY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        getFifthSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-pickupFifthSpecX, 144-pickupFifthSpecY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeFifthSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-placeFifthSpecX, 144-placeFifthSpecY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        parkTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(144-parkX,144-parkY,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();

    }
    public void autonomousPathUpdate() {
        switch(nextCase){
            case(1):
                letTimerRun = true;
                follower.followPath(preloadTraj);
                nextCase = 2;
                break;
            case(2):
                if(follower.getClosestPose().roughlyEquals(new Pose(34,66),5)){
                    armUpdate(INTAKE);
                }

                if(!follower.isBusy() && !(timer.getElapsedTimeSeconds()<WAIT_TIME)){
                    letTimerRun = false;

                    nextCase =3;

                }
                break;
            case(3):
                armUpdate(REST);
                letTimerRun = true;
                follower.followPath(firstSampTraj);
                nextCase = 4;
                break;
            case(4):
                if(follower.getClosestPose().roughlyEquals(new Pose(30,20),5)){
                    armUpdate(INTAKE);
                }
                if((((follower.getClosestPose().getX() == sample1X)&&follower.getClosestPose().getY()==sample1Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
                    letTimerRun = false;
                    nextCase = 5;
                }
                break;
            case(5):
                armUpdate(REST);
                letTimerRun = true;
                follower.followPath(placeSecondSpecTraj);
                nextCase = 6;
            case(6):

                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<WAIT_TIME))){
                    letTimerRun = false;

                    nextCase = 7;
                }
                break;
            case(7):
                letTimerRun = true;
                follower.followPath(secondSampTraj);
                nextCase = 9;
                break;
            case(9):

                if((((follower.getClosestPose().getX() == sample2X)&&follower.getClosestPose().getY()==sample2Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
                    letTimerRun = false;
                    nextCase = 10;
                }
                break;
            case(10):
                letTimerRun = true;
                follower.followPath(placeThirdSpecTraj);
                nextCase = 11;
                break;
            case(11):

                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<WAIT_TIME))){
                    letTimerRun = false;
                    nextCase = 12;
                }
                break;
            case(12):
                letTimerRun = true;
                follower.followPath(thirdSampTraj);
                nextCase = 13;
                break;
            case(13):
                if((((follower.getClosestPose().getX() == sample3X)&&follower.getClosestPose().getY()==sample3Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
                    letTimerRun = false;
                    nextCase = 14;
                }
                break;


            case(14):
                letTimerRun = true;
                follower.followPath(placeFourceSpecTraj);
                nextCase = 15;
                break;
            case(15):
                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<WAIT_TIME))){
                    letTimerRun = false;
                    nextCase = 16;
                }
                break;
            case(16):
                letTimerRun = true;
                follower.followPath(getFifthSpecTraj);
                nextCase = 17;
                break;
            case(17):
                if((((follower.getClosestPose().getX() == pickupFifthSpecX||follower.getClosestPose().getY()==pickupFifthSpecY) && (timer.getElapsedTimeSeconds()>WAIT_TIME)))){
                    letTimerRun = false;
                    nextCase = 10;
                }
                break;

            case(18):
                letTimerRun = true;
                follower.followPath(placeFifthSpecTraj);
                nextCase = 19;
                break;
            case(19):
                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<WAIT_TIME))){
                    letTimerRun = false;
                    nextCase = 20;
                }
                break;
            case(20):
                follower.followPath(parkTraj);
                nextCase=21;
                break;
            case(21):
                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<SAMP_TIME+4))){
                    letTimerRun = false;
                    nextCase = 404;
                }
                break;
            case(404):
                stop();



//
//
            default:
                break;

        }
    }


    public void armUpdate(int scenario){
        switch(scenario){
            case(INTAKE):
                //pivot.setPosition(PIVOT_WALL_INTAKE);
                //target = ARM_WALL_INTAKE;
                break;
            case(OUTTAKE):
                //pivot.setPostion(PIVOT_OUTTAKE);
                //target = ARM_OUTTAKE;
                break;
            case(REST):
                //pivot.setPosition(PIVOT_REST);
                //target = ARM_REST
        }

    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        telemetry.addLine("follower init");
        telemetry.update();
        pathBuilder();
        telemetry.addLine("path built");
        telemetry.update();
        follower.setStartingPose(new Pose(startX,startY, HEADING_WALL));
        telemetry.addLine("staring pos set");
        telemetry.update();
        timer = new Timer();
        letTimerRun = true;

    }

    @Override
    public void start() {

        telemetry.addLine("op mode started");
        telemetry.update();
        nextCase = 1;
        autonomousPathUpdate();


    }

    @Override
    public void loop() {

        follower.update();
        telemetry.addData("Time: ", timer.getElapsedTimeSeconds());
        telemetry.addData("isBusy(): ",follower.isBusy());
        telemetry.addData("at parametric end: ", follower.atParametricEnd());
        telemetry.addData("Closest pose: ", follower.getClosestPose());
        telemetry.update();
        if(!letTimerRun) {
            timer.resetTimer();
        }
        //arm PID update


        autonomousPathUpdate();






//telemetry.update();
    }

    public void stop(){
        requestOpModeStop();
    }
}
