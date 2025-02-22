package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.TEAM_COLOR;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.autoToTeleop;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_arm_pickup;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_arm_prep_pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.assets.SampleOrientationAnalysisPipeline;
import org.firstinspires.ftc.teamcode.aRROD.utils.UTILS;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class StateTeleop extends LinearOpMode {


    Gamepad currentAidenGamepad = new Gamepad();
    Gamepad previousAidenGamepad = new Gamepad();

    Gamepad currentJustinGamepad = new Gamepad();
    Gamepad previousJustinGamepad = new Gamepad();
    public static double speedMulti = 4;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleOrientationAnalysisPipeline angleFinder = new SampleOrientationAnalysisPipeline(telemetry);
        VisionPortal visionPortal;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(angleFinder)
                .enableLiveView(true)
                .build();
        //DcMotor motor = hardwareMap.dcMotor.get("motor_ch_3");
        PIDController extendo_controller = new PIDController(extendo_P, extendo_I, extendo_D);
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        Mechanisms mechs = new Mechanisms(hardwareMap, follower);
        ElapsedTime liftTimer = new ElapsedTime();
        ElapsedTime pickupTimer = new ElapsedTime();


        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        int extendo_transfer_toggle = 1000;
        int retry_toggle = 1000;
        int lift_toggle = 1000;
        int lift_pickup_toggle = 1000;
        int upDownToggle = 1000;
        double slowdown = 0.4;
        boolean manual = true;
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mechs.resetMotors();
        follower.setStartingPose(autoToTeleop);
        follower.startTeleopDrive();


        //UTILS.extendo_TARGET = 0;


        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            //sets up toggles
            previousAidenGamepad.copy(currentAidenGamepad);
            currentAidenGamepad.copy(gamepad1);
            previousJustinGamepad.copy(currentJustinGamepad);
            currentJustinGamepad.copy(gamepad2);



            if (gamepad1.right_trigger > 0.2) {
                slowdown = 0.4;
            } else {
                slowdown = 1;
            }

            //intake toggles
            if (currentJustinGamepad.dpad_up && !previousJustinGamepad.dpad_up) {
                extendo_transfer_toggle++;
                if (extendo_transfer_toggle > 2) {
                    extendo_transfer_toggle = 0;
                    retry_toggle = 2;
                }
                if (extendo_transfer_toggle == 0) {
                    mechs.extendo_arm_camera_pos();
                    retry_toggle = 1;
                }
                if (extendo_transfer_toggle == 1) {

                } else {
                    mechs.extendo_intake_transfer();

                }
                retry_toggle = -1;
            }

            if (currentJustinGamepad.dpad_left && !previousJustinGamepad.dpad_left) {
                upDownToggle++;
                if (upDownToggle > 1) {
                    upDownToggle = 0;

                }
                if (upDownToggle == 0) {
                    mechs.extendo_arm_prep_pos();

                }
                if (upDownToggle == 1) {
                    mechs.extendo_arm_intake();
                }



            }

            //retry intake toggle
            if (currentJustinGamepad.dpad_down && !previousJustinGamepad.dpad_down) {
                retry_toggle++;
                if (retry_toggle > 4) {
                    retry_toggle = 0;
                }
                if (retry_toggle == 0) {
                    mechs.extendo_arm_camera_pos();


                } else if (retry_toggle == 1) {
                    mechs.swivelMove(angleFinder.getAngleOfSample() / 180 * 0.6 + 0.2);
                } else if (retry_toggle == 2) {
                    mechs.extendo_arm_prep_pos();

                } else if (retry_toggle == 3) {
                    mechs.extendo_arm_intake();
                } else {
                    mechs.extendo_arm_secure();

                }


            }
            //closes extendo and readies for transfer
            if (currentAidenGamepad.left_bumper && !previousAidenGamepad.left_bumper) {
                lift_toggle++;
                if (lift_toggle > 4) {
                    lift_toggle = 0;
                }
                if (lift_toggle == 0) {
                    mechs.armIntake();

                    mechs.clawOpen();

                } else if (lift_toggle == 1) {
                    mechs.clawClosed();
                } else if (lift_toggle == 2) {
                    mechs.armScore();
                    mechs.liftIntermediate();

                } else if (lift_toggle == 3) {
                    mechs.lift_specimen();
                } else {
                    mechs.liftDown();
                    mechs.clawOpen();
                    mechs.armIntake();
                    lift_toggle = 0;


                }
            }


            if (currentJustinGamepad.right_bumper && !previousJustinGamepad.right_bumper) {
                lift_pickup_toggle++;
                if (lift_pickup_toggle > 3) {
                    lift_pickup_toggle = 0;
                }
                if (lift_pickup_toggle == 0) {

                    mechs.prepTransfer1();
                    manual = false;

                } else if (lift_pickup_toggle == 1) {
                    mechs.clawClosed();

                } else if (lift_pickup_toggle == 2) {


                    mechs.armIntake();
                    mechs.extendo_transfer_block();

                } else {

                    mechs.clawOpen();
                    mechs.extendo_arm_camera_pos();
                    retry_toggle = 0;
                    lift_toggle = 0;
                }
            }


            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * slowdown, -gamepad1.left_stick_x * slowdown, -gamepad1.right_stick_x * slowdown, false);

            follower.update();


//


            //manual control of shit
            if (Math.abs(currentJustinGamepad.left_stick_x) > 0.1) {
                mechs.turret_swivel(-gamepad2.left_stick_x * 1.1);
            }
            if (Math.abs(currentJustinGamepad.right_stick_x) > 0.1) {
                mechs.tape_swivel(gamepad2.right_stick_x * 1.1);
            }

            //manual control of extendo
            if (currentJustinGamepad.left_trigger > 0.3) {
                mechs.extendo_adjust(gamepad2.left_trigger * speedMulti);
                manual = true;

            }
            if (currentJustinGamepad.x) {
                speedMulti = 2;
            } else {
                speedMulti = 1;

            }
            if (currentJustinGamepad.right_trigger > 0.3) {
                manual = true;
                mechs.extendo_adjust(-gamepad2.right_trigger * speedMulti);

            }
            if (!manual) {
                mechs.extendoUpdate();
            }
            mechs.liftUpdate();
            telemetry.addData("retry toggle", retry_toggle);
            telemetry.update();


        }
    }
}





