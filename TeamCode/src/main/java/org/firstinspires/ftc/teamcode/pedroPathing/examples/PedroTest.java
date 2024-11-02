package org.firstinspires.ftc.teamcode.pedroPathing.examples;



import static java.lang.Thread.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@Autonomous(name = "Blue Close Side 2 + 4", group = "Autonomous")
public class BlueLeftInnerAuto extends OpMode {



    private Timer pathTimer, opmodeTimer, scanTimer, distanceSensorUpdateTimer, distanceSensorDecimationTimer;



    private VisionPortal visionPortal;

    private String navigation;

    private DistanceSensor leftDistanceSensor, rightDistanceSensor, rearDistanceSensor;

    private SingleRunAction foldUp;

    private boolean distanceSensorDisconnected, rearDistanceSensorDisconnected;

// IMPORTANT: y increasing is towards the backstage from the audience,
// while x increasing is towards the red side from the blue side
// this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
   private Point scorePreload = new Point(34,48);
   private Point firstSampleIntoObsPoint1 = new Point(24,36 );
   private Point firstSampleIntoObsPoint2 = new Point(52,28);
   private Point firstSampleIntoObsPoint3 = new Point(15,24);
   private Point placeSecondSpec = new Point(34,46);
    private Point secondSampleIntoObsPoint1 = new Point(24,36 );
    private Point secondSampleIntoObsPoint2 = new Point(52,28);
    private Point secondSampleIntoObsPoint3 = new Point(15,24);
    private Point placeThirdSpec = new Point(34,44);
    private Point park = new Point(10,10);


    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(144-(63+72), 12+72, 0);

    // TODO: dont forget to adjust this too
    private Point abortPoint = new Point(144-83.5, 120, Point.CARTESIAN), backdropGoalPoint;

    private Follower follower;

    private Path scoreSpikeMark, initialScoreOnBackdrop;
    private PathChain preload, firstSamp, secondSpec, secondSamp, thirdSpec, parktraj;

    private int pathState, distanceSensorDisconnectCycleCount, detectDistanceSensorDisconnect;

    private ArrayList<Boolean> distanceSensorDisconnects;

    public void setBackdropGoalPose() {

    }

    public void buildPaths() {
        Point scoreSpikeMarkMidPoint;
        double scoreSpikeMarkMidToSpikeDistance;
        preload = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(),scorePreload))
                .build()

        firstCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(firstCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        firstCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(firstCycleStackPose)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .build();

        firstCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstCycleStackPose), new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(firstCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(firstCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(firstCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build();

        secondCycleToStack = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(firstCycleBackdropGoalPose), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(secondCycleStackPose.getX(), 23, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        secondCycleStackGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose.getX()+0.0001, 32, Point.CARTESIAN), new Point(secondCycleStackPose)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .build();

        secondCycleScoreOnBackdrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondCycleStackPose), new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(secondCycleStackPose.getHeading())
                .addPath(new BezierCurve(new Point(secondCycleStackPose.getX()+2, 79, Point.CARTESIAN), new Point(144-76.5, 106, Point.CARTESIAN), new Point(secondCycleBackdropGoalPose)))
                .setConstantHeadingInterpolation(Math.PI * 1.5)
                .setPathEndTimeoutConstraint(2.5)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10: // scores preload
                follower.followPath(scorePreload);
                setPathState(11);
                break;
            case 11: // detects the path to progress away from the wall and sets tangent interpolation
                if (follower.getCurrentTValue() > 0.1) {
//scoreSpikeMark.setReversed(false);
                    scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading() - 0.1 * MathFunctions.getTurnDirection(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()) * MathFunctions.getSmallestAngleDifference(startPose.getHeading(), scoreSpikeMark.getEndTangent().getTheta()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.holdPoint(new BezierPoint(scoreSpikeMark.getLastControlPoint()), scoreSpikeMark.getEndTangent().getTheta());
                    setPathState(13);
                }
                break;
            case 13: // detects for the end of the path and everything else to be in order and releases the pixel
                if (twoPersonDrive.intakeState == INTAKE_OUT) {
                    twoPersonDrive.setIntakeClawOpen(true);
                    setPathState(14);
                }
                break;
            case 14: // moves mechanisms into position to score and pick up from stack as well as starts moving to score
                if (pathTimer.getElapsedTime() > INTAKE_CLAW_CLOSE_TIME) {
                    twoPersonDrive.setTransferState(TRANSFER_OUT);
                    twoPersonDrive.outtakeWristOffset = -15;
                    twoPersonDrive.moveToCustomIntakeOutPosition(INTAKE_ARM_AUTO_AVOID_POSITION);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 500) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(16);
                }
                break;
            case 16: // detects for end of the path and outtake out and drops pixel
                if (!follower.isBusy() && twoPersonDrive.outtakeState == OUTTAKE_OUT) {
                    Follower.useHeading = true;
                    backdropGoalPoint = new Point(initialBackdropGoalPose);
                    follower.holdPoint(new BezierPoint(backdropGoalPoint), Math.PI * 1.5);
                    twoPersonDrive.setOuttakeArmInterpolation(OUTTAKE_ARM_YELLOW_SCORE_POSITION);
                    distanceSensorDecimationTimer.resetTimer();
                    startDistanceSensorDisconnectDetection(-1);
                    setPathState(17);
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void stackCorrection(double correctionPower) {
        if (distanceSensorDecimationTimer.getElapsedTime() > 20) {

            double left = leftDistanceSensor.getDistance(DistanceUnit.MM);

            if (!(left == 65535)) {

                double right = rightDistanceSensor.getDistance(DistanceUnit.MM);

                if (!(right == 65535)) {

                    double error = (left / 25.4) - (right / 25.4);

                    error *= -1;

                    if (Math.abs(error) > 0.5) {
                        follower.setXOffset(follower.getXOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * correctionPower * MathFunctions.getSign(error));
                    } else {
                        follower.setXOffset(follower.getXOffset() + follower.getTranslationalError().getXComponent());
                    }

                    follower.setXOffset(MathFunctions.clamp(follower.getXOffset(), -6, 6));

//telemetry.addData("error", error);
                    distanceSensorDecimationTimer.resetTimer();
                } else {
                    distanceSensorDisconnected = true;
                }
            } else {
                distanceSensorDisconnected = true;
            }
        }
    }

    public boolean leftDistanceSensorDisconnected() {
        return leftDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
    }

    public boolean rightDistanceSensorDisconnected() {
        return rightDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
    }

    public void startDistanceSensorDisconnectDetection(int state) {
        detectDistanceSensorDisconnect = 0;//state;
        distanceSensorDisconnectCycleCount = 0;
        distanceSensorDisconnects.clear();
    }

    public void updateDistanceSensorDisconnects() {
        if (detectDistanceSensorDisconnect == 1) {
            if (distanceSensorDisconnectCycleCount < 10) {
                if (distanceSensorUpdateTimer.getElapsedTime() > 20) {
                    distanceSensorDisconnectCycleCount++;
                    distanceSensorUpdateTimer.resetTimer();

                    distanceSensorDisconnects.add(leftDistanceSensorDisconnected() || rightDistanceSensorDisconnected());
                }
            } else {
                detectDistanceSensorDisconnect = 0;

                distanceSensorDisconnected = true;
                for (Boolean detection : distanceSensorDisconnects) {
                    if (!detection) {
                        distanceSensorDisconnected = false;
                    }
                }
            }
        } else if (detectDistanceSensorDisconnect == -1) {
            if (distanceSensorDisconnectCycleCount < 10) {
                if (distanceSensorUpdateTimer.getElapsedTime() > 20) {
                    distanceSensorDisconnectCycleCount++;
                    distanceSensorUpdateTimer.resetTimer();

                    distanceSensorDisconnects.add(rearDistanceSensorDisconnected());
                }
            } else {
                detectDistanceSensorDisconnect = 0;

                rearDistanceSensorDisconnected = true;
                for (Boolean detection : distanceSensorDisconnects) {
                    if (!detection) {
                        rearDistanceSensorDisconnected = false;
                    }
                }
            }
        }
    }

    public void backdropCorrection(Pose scorePose, double distanceGoal) {
        if (distanceSensorDecimationTimer.getElapsedTime() > 20) {

            double distance = rearDistanceSensor.getDistance(DistanceUnit.MM);

            if (distance != 65535) {
//follower.holdPoint(new BezierPoint(new Point(scorePose.getX(), MathFunctions.clamp(follower.getPose().getY() + (distance / 25.4) - distanceGoal, scorePose.getY() - 4, scorePose.getY() + 4), Point.CARTESIAN)), Math.PI * 1.5);
                backdropGoalPoint.setCoordinates(scorePose.getX(), MathFunctions.clamp(follower.getPose().getY() + ((distance / 25.4) - distanceGoal), scorePose.getY() - 4, scorePose.getY() + 4), Point.CARTESIAN);
            } else {
                rearDistanceSensorDisconnected = true;
            }
/*
// too close
if (distance < 0.5)
follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() + distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);

// too far
if (distance > 0.75)
follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - distanceSensorDecimationTimer.getElapsedTimeSeconds() * 1.5);

// to do add some sort of deadzone or dampening
// perhaps take note of the estimated pose at the start and see how far off we need to go instead of incrementing off of the current one
// or just remove the getyoffset thing? think about later
follower.poseUpdater.setYOffset(follower.poseUpdater.getYOffset() - (distance - 2));

if (Math.abs(follower.poseUpdater.getYOffset()) > 1.5)
follower.poseUpdater.setYOffset(1.5 * MathFunctions.getSign(follower.poseUpdater.getYOffset()));
*/
//telemetry.addData("rear distance value", distance);
            distanceSensorDecimationTimer.resetTimer();
        }
    }

    public boolean rearDistanceSensorDisconnected() {
        return rearDistanceSensor.getDistance(DistanceUnit.MM) == 65535;
    }

    @Override
    public void loop() {

        follower.update();


        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
//telemetry.update();
    }

    @Override
    public void init() {
//PhotonCore.start(this.hardwareMap);

        foldUp = new SingleRunAction(()-> {
            if (Integer.parseInt(String.valueOf(pathState).substring(0,1)) < 4) setPathState(40);
        });


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();
        distanceSensorUpdateTimer = new Timer();
        distanceSensorDecimationTimer = new Timer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);



//        visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
//                .addProcessors(teamPropPipeline)
//                .setCameraResolution(new Size(640, 480))
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .setAutoStopLiveView(true)
//                .build();



        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        if (leftDistanceSensorDisconnected()) {
            try {
                throw new Exception("left distance sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        if (rightDistanceSensorDisconnected()) {
            try {
                throw new Exception("right distance sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }

        if (rearDistanceSensorDisconnected()) {
            try {
                throw new Exception("color sensor disconnected");
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }



        try {
            Thread.sleep(3000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }



        try {
            sleep(2500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scanTimer.resetTimer();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}