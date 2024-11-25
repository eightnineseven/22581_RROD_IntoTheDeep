package org.firstinspires.ftc.teamcode.aRROD.Auto;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armIntake;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armRest;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armScore;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.clawClosed;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.firstSample1WeightX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.firstSample1WeightY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.firstSample2WeightX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.firstSample2WeightY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.parkX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.parkY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pickupFifthSpecX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pickupFifthSpecY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pivotHolding;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeFifthSpecX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeFifthSpecY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeFourthSpecX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeFourthSpecY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeSecondSpecX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeSecondSpecY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeThirdSpecX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.placeThirdSpecY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.preloadPlaceX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.preloadPlaceY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample1X;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample1Y;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample2X;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample2Y;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample3X;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.sample3Y;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.secondSample1WeightX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.secondSample1WeightY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.secondSample2WeightX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.secondSample2WeightY;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.startX;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.startY;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="BlueDebug",group="aRROD")
public class BlueDebug extends OpMode {



    public Follower follower;
    private int pathState;
    private int nextCase;
    public Timer timer;
    public boolean letTimerRun = false;
    public double WAIT_TIME=2.7;
    public double SAMP_TIME = 3;
    public double ff;
   public static double HEADING_WALL = Math.toRadians(180);
   public static double HEADING_SUB = 0;
    private final int INTAKE = -2;
   private final int OUTTAKE = -1;
   private final int REST = -3;
   public static int oldTarget = -1;
    public boolean pathJustSwitched = false;
    public boolean autoFinished = false;
    public boolean updateSubsystems = false;
    public int nextRobotAction;
    public static Timer armTimer = new Timer();
    public static Servo clawL;
    public static Servo clawR;
    public static Servo clawPivot;
    public static DcMotor arm_motor;
    public static int target=0;
    public static PIDController controller;
    public static int LAST_USED = 0;
    public static boolean letRestTimerRun;
    public static Timer restTimer = new Timer();


    private PathChain preloadTraj, firstSampTraj, firstSampPushTraj, placeSecondSpecTraj, secondSampTraj, placeThirdSpecTraj, thirdSampTraj, placeFourceSpecTraj, getFifthSpecTraj, placeFifthSpecTraj, parkTraj;

    public void pathBuilder(){
        preloadTraj = follower.pathBuilder()
                .addBezierLine(new Point(startX, startY), new Point(preloadPlaceX, preloadPlaceY))
                .setConstantHeadingInterpolation(HEADING_SUB)
                .build();
        firstSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(sample1X, sample1Y, Point.CARTESIAN), new Point(firstSample1WeightX, firstSample1WeightY, Point.CARTESIAN), new Point(secondSample1WeightX, secondSample1WeightY, Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_WALL)
                .addBezierCurve(new Point(sample1X,sample1Y,Point.CARTESIAN), new Point(sample1X+0.1,sample1Y+0.1,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        firstSampPushTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),new Point(7,7))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeSecondSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),new Point(placeSecondSpecX,placeSecondSpecY,Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), HEADING_SUB)
                .build();
        secondSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(sample2X, sample2Y, Point.CARTESIAN), new Point(firstSample2WeightX, firstSample2WeightY, Point.CARTESIAN), new Point(secondSample2WeightX, secondSample2WeightY, Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),HEADING_WALL)
                .setPathEndHeadingConstraint(0.07)
                .addBezierCurve(new Point(sample2X,sample2Y,Point.CARTESIAN),new Point(sample2X,sample2Y,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .setPathEndHeadingConstraint(0.07)
                .build();
        placeThirdSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeThirdSpecX, placeThirdSpecY, Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),HEADING_SUB)
                .build();

        thirdSampTraj = follower.pathBuilder()
                .addBezierCurve(new Point(sample3X,sample3Y,Point.CARTESIAN),new Point(sample3X,sample3Y,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeFourceSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeFourthSpecX, placeFourthSpecY, Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),HEADING_SUB)
                .build();
        getFifthSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(pickupFifthSpecX, pickupFifthSpecY, Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_WALL)
                .build();
        placeFifthSpecTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(placeFifthSpecX, placeFifthSpecY, Point.CARTESIAN))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(),HEADING_SUB)
                .build();
        parkTraj = follower.pathBuilder()
                .addBezierLine(new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN), new Point(parkX,parkY,Point.CARTESIAN))
                .setConstantHeadingInterpolation(HEADING_SUB)
                .build();

    }
    public void autonomousPathUpdate() {
        switch(nextCase){
            case(1):
                letTimerRun = false;
                follower.followPath(preloadTraj);
                nextCase = 2;

                break;
            case(2):
                letTimerRun = true;
                if(!follower.isBusy()){
                    target = (int)armScore;

                    while(!armUpdate()&&timer.getElapsedTimeSeconds()<3){
                        telemetry.update();
                        follower.update();
                    }
                    nextCase = 3;


                }
                break;
            case(3):
                timer.resetTimer();
                nextCase=4;
                letTimerRun = true;
                break;
            case(4):
                target = (int)armRest;
                while(!armUpdate()&&timer.getElapsedTimeSeconds()<1){
                    telemetry.update();
                    follower.update();
                }
                arm_motor.setPower(0);
                nextCase = 5;
                break;
            case(5):
                follower.followPath(firstSampTraj);
                letTimerRun = false;
                follower.update();
                nextCase = 6;
                follower.update();
                break;
            case(6):
                letTimerRun = true;
                follower.update();
                if((((follower.getClosestPose().getX() == sample1X)&&follower.getClosestPose().getY()==sample1Y) || !follower.isBusy())){
                    nextCase=7;
                    letTimerRun = false;
                }
                break;
            case(7):
                letTimerRun = true;
                    target = (int)armIntake;
                    while(!armUpdate()&&timer.getElapsedTimeSeconds()<3) {
                        telemetry.update();
                        follower.update();
                    }
                    nextCase = 8;
                    letTimerRun = false;


                break;
            case(8):
                letTimerRun = true;
                target = (int)armRest;
                while(!armUpdate()&&timer.getElapsedTimeSeconds()<1){
                    telemetry.update();
                    follower.update();
                }
                arm_motor.setPower(0);
                nextCase = 9;
                break;

            case(9):
                nextCase=404;
                break;

//            case(4):
//                if(follower.getClosestPose().roughlyEquals(new Pose(10,20),4)){
//                    armUpdate(INTAKE);
//                }
//                if((((follower.getClosestPose().getX() == sample1X)&&follower.getClosestPose().getY()==sample1Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
//                    letTimerRun = false;
//                    nextCase = 5;
//                }
//                break;
//            case(5):
//                while(armTimer.getElapsedTimeSeconds()<3){
//                    follower.update();
//                    telemetry.update();
//                }
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(placeSecondSpecTraj);
//                nextCase = 6;
//            case(6):
//                if(follower.getClosestPose().roughlyEquals(new Pose(34,66),8)){
//                    armUpdate(OUTTAKE);
//                }
//                if((!follower.isBusy())){
//                    letTimerRun = false;
//
//                    nextCase = 7;
//                }
//                break;
//            case(7):
//                while(armTimer.getElapsedTimeSeconds()<3){
//                    follower.update();
//                    telemetry.update();
//                }
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(secondSampTraj);
//                nextCase = 9;
//                break;
//            case(9):
//                if(follower.getClosestPose().roughlyEquals(new Pose(12,12),12)){
//                    armUpdate(INTAKE);
//                }
//
//                if((((follower.getClosestPose().getX() == sample2X)&&follower.getClosestPose().getY()==sample2Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
//                    letTimerRun = false;
//                    nextCase = 10;
//                }
//                break;
//            case(10):
//                armTimer.resetTimer();
//                while(armTimer.getElapsedTimeSeconds()<3){
//                    follower.update();
//                    telemetry.update();
//                }
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(placeThirdSpecTraj);
//                nextCase = 11;
//                break;
//            case(11):
//                if(follower.getClosestPose().roughlyEquals(new Pose(34,66),8)){
//                    armUpdate(OUTTAKE);
//                }
//                if((!follower.isBusy())){
//                    letTimerRun = false;
//                    nextCase = 12;
//                }
//                break;
//            case(12):
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(thirdSampTraj);
//                nextCase = 13;
//                break;
//            case(13):
//                if(follower.getClosestPose().roughlyEquals(new Pose(12,12),12)){
//                    armUpdate(INTAKE);
//                }
//                if((((follower.getClosestPose().getX() == sample3X)&&follower.getClosestPose().getY()==sample3Y) && (timer.getElapsedTimeSeconds()>SAMP_TIME))){
//                    letTimerRun = false;
//                    nextCase = 14;
//                }
//                break;
//
//
//            case(14):
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(placeFourceSpecTraj);
//                nextCase = 15;
//                break;
//            case(15):
//                if(follower.getClosestPose().roughlyEquals(new Pose(34,66),8)){
//                    armUpdate(OUTTAKE);
//                }
//                if((!follower.isBusy())){
//                    letTimerRun = false;
//                    nextCase = 16;
//                }
//                break;
//            case(16):
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(getFifthSpecTraj);
//                nextCase = 17;
//                break;
//            case(17):
//                if(follower.getClosestPose().roughlyEquals(new Pose(12,12),12)){
//                    armUpdate(INTAKE);
//                }
//                if((((follower.getClosestPose().getX() == pickupFifthSpecX||follower.getClosestPose().getY()==pickupFifthSpecY) && (timer.getElapsedTimeSeconds()>WAIT_TIME)))){
//                    letTimerRun = false;
//                    nextCase = 10;
//                }
//                break;
//
//            case(18):
//                armUpdate(REST);
//                letTimerRun = true;
//                follower.followPath(placeFifthSpecTraj);
//                nextCase = 19;
//                break;
//            case(19):
//                if(follower.getClosestPose().roughlyEquals(new Pose(34,66),8)){
//                    armUpdate(OUTTAKE);
//                }
//                if((!follower.isBusy())){
//                    letTimerRun = false;
//                    nextCase = 20;
//                }
//                break;
//            case(20):
//                armUpdate(REST);
//                follower.followPath(parkTraj);
//                nextCase=21;
//                break;
//            case(21):
//                if((!follower.isBusy() && !(timer.getElapsedTimeSeconds()<SAMP_TIME+5))){
//                    letTimerRun = false;
//                    nextCase = 404;
//                }
//                break;
            case(404):
                stop();
            break;



//
//
            default:
                break;

        }
}


   public boolean armUpdate(){
       controller.setPID(ARM_P,ARM_I,ARM_D);
       int armPos = arm_motor.getCurrentPosition();
       double pid = controller.calculate(armPos, target);
       double ff = Math.cos(Math.toRadians(target / ARM_TICKS_IN_DEGREES)) * ARM_F;
       double power = pid + ff;
       follower.update();
       if(target==arm_motor.getCurrentPosition()) {
           oldTarget = target;
           arm_motor.setPower(ff);
           return true;

       } else if(target!=oldTarget){
           arm_motor.setPower(power);
           telemetry.addLine("power");
       }
       telemetry.addData("arm current pos: ", arm_motor.getCurrentPosition());
       telemetry.addData("target pos: ", target);

    return false;
   }

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        telemetry.addLine("follower init");
        telemetry.update();
        pathBuilder();
        telemetry.addLine("path built");
        telemetry.update();
        follower.setStartingPose(new Pose(startX,startY, HEADING_SUB));
        telemetry.addLine("staring pos set");
        telemetry.update();
        timer = new Timer();
        letTimerRun = true;
         clawR = hardwareMap.get(Servo.class, "servo_ch_1");
         clawL = hardwareMap.get(Servo.class, "servo_ch_2");
         clawPivot = hardwareMap.get(Servo.class, "servo_ch_0");
         clawR.setDirection(Servo.Direction.REVERSE);
        arm_motor = hardwareMap.get(DcMotor.class, "motor_eh_0");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawR.setPosition(clawClosed);
        clawL.setPosition(clawClosed);
        timer.resetTimer();
        while(timer.getElapsedTimeSeconds()<2){
            follower.update();
            telemetry.update();
        }
        clawPivot.setPosition(pivotHolding);
        controller = new PIDController(ARM_P,ARM_I,ARM_D);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void start() {

        telemetry.addLine("op mode started");
        telemetry.update();
        nextCase = 1;
        autonomousPathUpdate();
        armTimer.resetTimer();
        restTimer.resetTimer();
        timer.resetTimer();


    }

    @Override
    public void loop() {
        follower.update();
        if(!letTimerRun){
            timer.resetTimer();
        }
        autonomousPathUpdate();
        if(!letTimerRun){
            timer.resetTimer();
        }






//telemetry.update();
    }

    public void stop(){
            requestOpModeStop();
    }
}
