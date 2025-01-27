package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;

@TeleOp
@Config
public class RegionalsTeleop extends LinearOpMode {


    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() throws InterruptedException {


        //DcMotor motor = hardwareMap.dcMotor.get("motor_ch_3");
        PIDController extendo_controller = new PIDController(extendo_P, extendo_I, extendo_D);
        Mechanisms mechs = new Mechanisms(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        int extendo_transfer_toggle = 0;
        int retry_toggle = 0;
        int lift_toggle = 0;
        int lift_pickup_toggle = 0;
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mechs.resetMotors();
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            //sets up toggles
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            //intake toggles
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                extendo_transfer_toggle++;
                if (extendo_transfer_toggle > 2) {
                    extendo_transfer_toggle = 0;
                }
                if (extendo_transfer_toggle == 0) {
                    mechs.extendo_arm_camera_pos();
                } else if (extendo_transfer_toggle == 1) {
                    mechs.extendo_arm_prep_pos();
                    retry_toggle = 0;
                } else {
                    mechs.extendo_intake_transfer();
                    mechs.extendoClose();

                }
            }
            //retry intake toggle
            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
                retry_toggle++;
                if (retry_toggle > 1) {
                    retry_toggle = 0;
                }
                if(retry_toggle == 0){

                    mechs.extendo_arm_prep_pos();
                } else {
                    mechs.extendo_arm_intake();

                }
            }
            //closes extendo and readies for transfer
            if (currentGamepad.left_bumper && !previousGamepad.left_bumper) {
                lift_toggle++;
                if (lift_toggle > 4) {
                    lift_toggle = 0;
                }
                if(lift_toggle == 0){
                    mechs.armIntake();

                } else if(lift_toggle==1){
                    mechs.clawClosed();
                } else if(lift_toggle ==2){
                    mechs.armScore();
                    mechs.extendo_arm_prep_pos();
                } else if(lift_toggle ==3){
                    mechs.lift_specimen();
                } else {
                    mechs.liftDown();
                    mechs.clawOpen();
                    
                }
            }



            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) {
                lift_pickup_toggle++;
                if (lift_pickup_toggle > 3) {
                    lift_pickup_toggle = 0;
                }
                if(lift_pickup_toggle == 0){
                    mechs.prepTransfer();

                } else if(lift_pickup_toggle==1){
                    mechs.clawClosed();
                } else if(lift_pickup_toggle ==2){
                    mechs.armIntake();

                } else {
                    mechs.liftDown();
                    mechs.clawOpen();
                    lift_toggle = 0;
                }
            }




            //manual control of shit
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                mechs.turret_swivel(-gamepad1.left_stick_x);
            }
            if (Math.abs(gamepad1.right_stick_x) > 0.1) {
                mechs.tape_swivel(-gamepad1.right_stick_x);
            }

            //manual control of extendo
            if (gamepad1.left_trigger > 0.1) {
                mechs.extendo_out(gamepad1.left_trigger);
            }
            if (gamepad1.right_trigger > 0.1) {
                mechs.extendo_in_manual(gamepad1.right_trigger);
            }




//            //PID controls
            mechs.extendoUpdate();
            mechs.liftUpdate();


            telemetry.addData("lift caseL:  ", lift_toggle);

            telemetry.update();


        }
    }
}

