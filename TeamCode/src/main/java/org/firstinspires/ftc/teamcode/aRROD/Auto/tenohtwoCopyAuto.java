//package org.firstinspires.ftc.teamcode.aRROD.Auto;
//
//import com.pedropathing.util.Constants;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.List;
//
//
//import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
//import org.firstinspires.ftc.teamcode.aRROD.assets.PathGenerator;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
//import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.pedropathing.util.Drawing;
//
//@Autonomous(name = "fiveSpec", group = "auto", preselectTeleOp = "Teleop")
//public class tenohtwoCopyAuto extends OpMode {
//    private ElapsedTime timer = new ElapsedTime();
//    private Follower follower;
//    private Timer pathTimer;
//    private Mechanisms mechs;
//    private PathGenerator pathGen;
//    private int pathState;
//
//    private PathChain preload, pushSample1Path, pushSample2Path, pushSample3Path, combinedPush, score1, return1, score2, return2, score3, return3, score4, return4, shift, inter;
//    private Pose startingPose =  new Pose   (7, 65, Math.toRadians(0));
//
//    private double wall_intake = 7.5;
//    private double subX = 39;
//
//
//    /**
//     * Build our complex path sequence using the same calls
//     * that were in the GeneratedPath constructor.
//     */
//
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                if(!follower.isBusy()) {
//                    // Start following our newly built path
//                    follower.followPath(preload, 1, true);
//                    mechs.liftUpdate();
//                    mechs.extendoUpdate();
//
//                }
//                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 1.5) {
//                    deposit.setSlideTarget(500);
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    deposit.setSlideTarget(230);
//                    if (deposit.liftPos < 250) {
//                        endEffector.openClaw();
//                        setPathState(pathState + 1);
//                    }
//                }
//                break;
//            case 1:
//                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
//                    deposit.setSlideTarget(0);
//                    deposit.setPivotTarget(90);
//                    endEffector.setWallIntakePositionAlt();
//                    follower.followPath(combinedPush, 1,false);
//                    setPathState();
//                }
//                break;
//            case 2:
//                if(!follower.isBusy()) {
//                    follower.followPath(inter,0.6, true);
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.2) {
//                    endEffector.closeClaw();
//                    deposit.setSlideTarget(50);
//                    setPathState(5);
//                }
//                break;
//            case 5:
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    follower.followPath(score1, true); // Follow score1 path
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
//                    deposit.setSlideTarget(110);
//                    endEffector.setSpecScore(); // Adjust claw for scoring
//                    // Set initial slide position
//                }
//
//
//                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
//                    deposit.setSlideTarget(460); // Lift slide to scoring position
//                }
//
//                if (pathTimer.getElapsedTimeSeconds() > 3) {
//                    deposit.setSlideTarget(230); // Lower slide slightly for object release
//                    if (deposit.liftPos < 250) {
//                        endEffector.openClaw(); // Release object
//                        setPathState(pathState + 1); // Transition to return state
//                    }
//                }
//                break;
//
//            case 6:
//                if (!follower.isBusy()) {
//                    deposit.setSlideTarget(0); // Reset slide to initial position
//                    deposit.setPivotTarget(90); // Reset pivot
//                    endEffector.setWallIntakePositionAlt(); // Prepare claw for intake
//                    follower.followPath(return1, true); // Follow return1 path
//                    setPathState(pathState + 1); // Transition to the next scoring path
//                }
//                break;
//            case 7:
//                cycle();
//                break;
//            case 8:
//                intake();
//                break;
//            case 9:
//                cycle();
//                break;
//            case 10:
//                intake();
//                break;
//            case 11:
//                cycle();
//                if (pathTimer.getElapsedTimeSeconds() > 4.4) {
//                    deposit.setSlideTarget(230);
//                    if (deposit.liftPos < 250) {
//                        endEffector.openClaw();
//                        setPathState(pathState + 1); // Transition to next return
//                    }
//                }
//                break;
//            case 12:
//                setPathState(-1);
//                break;
//            default:
//                if (!follower.isBusy()) {
//                    requestOpModeStop(); // Stop the OpMode if all states are complete
//                }
//                break;
//        }
//    }
//
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//
//    public void setPathState() {
//        pathState += 1;
//        pathTimer.resetTimer();
//    }
//
//    private void cycle() {
//
//        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
//            follower.followPath(score2, true); // Follow score2 path
//        }
//        if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.3) {
//            endEffector.closeClaw();
//            deposit.setSlideTarget(110);
//        }
//
//        if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
//            endEffector.setSpecScore();
//            deposit.setSlideTarget(100);
//        }
//
//        if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.3) {
//            deposit.setSlideTarget(460);
//        }
//
//        if (pathTimer.getElapsedTimeSeconds() > 4.6) {
//            deposit.setSlideTarget(230);
//            if (deposit.liftPos < 250) {
//                endEffector.openClaw();
//                setPathState(pathState + 1); // Transition to next return
//            }
//        }
//    }
//
//    private void intake() {
//        if (!follower.isBusy() || follower.getVelocityMagnitude() < 0.05) {
//            deposit.setSlideTarget(0);
//            deposit.setPivotTarget(90);
//            endEffector.closeClaw();
//            endEffector.setWallIntakePositionAlt();
//            follower.followPath(return2, false); // Follow return2 path
//            setPathState(pathState + 1); // Transition to next scoring path
//        }
//    }
//
//    private void preCycle() {
//        if (!follower.isBusy()) {
//            endEffector.closeClaw();
//            deposit.setSlideTarget(50);
//            setPathState(pathState + 1);
//        }
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetry.addData("X Pos", follower.getPose().getX());
//        telemetry.addData("Y Pos", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//        telemetry.addData("Path State", pathState);
//        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("Path Busy", follower.isBusy());
//
//
//        telemetry.update();
//
//
//
//        deposit.update();
//
//        autonomousPathUpdate();
//    }
//
//    @Override
//    public void init() {
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        pathTimer = new Timer();
//
//        // Initialize follower, slides, etc. as usual
//        Constants.setConstants(FConstants.class, LConstants.class);
//        follower = new Follower(hardwareMap);
//
//        // If needed, set a starting pose (only if your system requires it)
//        // follower.setStartingPose(new Pose(0,0, 0));
//        follower.setStartingPose(startingPose);
//        follower.setMaxPower(1);
//
//        deposit = new Deposit(hardwareMap, telemetry, true);
//        endEffector = new EndEffector(hardwareMap);
//
//        endEffector.setSpecScore();
//        deposit.setPivotTarget(90);
//        deposit.setSlideTarget(50);
//
//        // Build our newly incorporated multi-step path:
//        buildPaths();
//
//        if (!deposit.slideLimit.isPressed()) {
//            throw new IllegalArgumentException("Zero slides before init");
//        }
//    }
//
//    @Override
//    public void start() {
//        endEffector.setSpecScore();
//        pathTimer.resetTimer();
//        setPathState(0);
//    }
//
//}
