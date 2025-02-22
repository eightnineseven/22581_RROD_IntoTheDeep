package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.autoToTeleop;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.assets.SampleOrientationAnalysisPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class RegionalsTeleop extends LinearOpMode {


    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {


        //DcMotor motor = hardwareMap.dcMotor.get("motor_ch_3");
        PIDController extendo_controller = new PIDController(extendo_P, extendo_I, extendo_D);
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        Mechanisms mechs = new Mechanisms(hardwareMap, follower);
        ElapsedTime liftTimer = new ElapsedTime();
        ElapsedTime pickupTimer = new ElapsedTime();
        double scoreX = 34;
        double scoreY = 78;
        double pickupX = 6.8;
        double pickupy = 27;
        boolean wasLastScore = false;
        boolean scoringCycle = false;


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        int extendo_transfer_toggle = 1000;
        int retry_toggle = 1000;
        int lift_toggle = 1000;
        int lift_pickup_toggle = 1000;
        int claw_toggle = 1000;
        double slowdown = 0.4;
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mechs.resetMotors();
        follower.setStartingPose(autoToTeleop);
        follower.startTeleopDrive();
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            //sets up toggles
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            if(gamepad1.right_trigger>0.2){
                slowdown = 0.4;
            } else {
                slowdown = 1;
            }

            //intake toggles
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                extendo_transfer_toggle++;
                if (extendo_transfer_toggle > 2) {
                    extendo_transfer_toggle = 0;
                }
                if (extendo_transfer_toggle == 0) {
                    mechs.extendo_arm_prep_pos();
                    retry_toggle = 0;
                }if(extendo_transfer_toggle == 1){
                    mechs.retryPrepArm();
                } else {
                    mechs.extendo_intake_transfer();

                }
            }





            if (currentGamepad2.b && !previousGamepad2.b) {
                claw_toggle++;
                if (claw_toggle > 1) {
                    claw_toggle = 0;
                }
                if (claw_toggle == 0) {
                    mechs.clawOpen();

                }else {
                    mechs.clawClosed();
                }

            }





            //retry intake toggle
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                retry_toggle++;
                if (retry_toggle > 1) {
                    retry_toggle = 0;
                }
                if(retry_toggle == 0){

                    mechs.retryPrepArm();
                } else {
                    mechs.extendo_arm_intake();

                }
            }
            //closes extendo and readies for transfer
            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                lift_toggle++;
                if (lift_toggle > 5) {
                    lift_toggle = 0;
                }
                if(lift_toggle == 0){
                    mechs.armIntake();
                    mechs.clawOpen();

                } else if(lift_toggle==1){
                    mechs.clawClosed();
                }
                else if(lift_toggle ==2) {
                    mechs.armScore();

                } else if(lift_toggle==3){
                        mechs.liftIntermediate();
                } else if(lift_toggle ==4){
                    mechs.lift_specimen();
                } else {
                    mechs.liftDown();
                    mechs.clawOpen();
                    
                }
            }
            if(currentGamepad2.b&&!previousGamepad2.b){
                mechs.resetMotors();
            }



            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                lift_pickup_toggle++;
                if (lift_pickup_toggle > 4) {
                    lift_pickup_toggle = 0;
                }
                if(lift_pickup_toggle == 0){
                    mechs.prepTransfer1();

                } else if(lift_pickup_toggle==1){
                        mechs.prepTransfer2();
                } else if(lift_pickup_toggle ==2){
                    mechs.clawClosed();

                } else if(lift_pickup_toggle==3){
                    mechs.armIntake();

                } else{
                    mechs.liftDown();
                    mechs.clawOpen();
                    lift_toggle = 0;
                }
            }
            
//            if(!follower.isBusy()){
//                follower.startTeleopDrive();
//                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
//                follower.update();
//                if(wasLastScore){
//                    if(gamepad1.left_bumper){
//                        scoringCycle = true;
//                        liftTimer.reset();
//
//
//                    }
//                }
//            }
//            if(scoringCycle){
//                mechs.lift_specimen();
//                if(mechs.liftCloseEnough()||liftTimer.seconds()>0.3){
//                    mechs.clawOpen();
//                    mechs.liftDown();
//                }
//                scoringCycle = false;
//            }
//            if(currentGamepad1.dpad_right&&!previousGamepad1.dpad_right){
//                scoreY -= 2;
//                follower.followPath(new Path(
//
//                        new BezierLine(
//                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                                new Point(scoreX, scoreY)
//                        )
//                ));
//                wasLastScore = true;
//                scoringCycle = true;
//            }
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*slowdown, -gamepad1.left_stick_x*slowdown, -gamepad1.right_stick_x*slowdown, false);

            follower.update();


//            if(currentGamepad1.dpad_left&&!previousGamepad1.dpad_left){
//                scoreY -= 2;
//                follower.followPath(new Path(
//
//                        new BezierLine(
//                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                                new Point(pickupX, pickupy)
//                        )
//
//                ));
//                wasLastScore = false;
//                scoringCycle = false;
//            }




            //manual control of shit
            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                mechs.turret_swivel(-gamepad2.left_stick_x*1.1);
            }
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                mechs.tape_swivel(-gamepad2.right_stick_x*1.1);
            }

            //manual control of extendo



//            //PID controls
            mechs.extendoUpdate();
            mechs.liftUpdate();


            telemetry.addData("lift caseL:  ", lift_toggle);

            telemetry.update();


        }
    }
}


