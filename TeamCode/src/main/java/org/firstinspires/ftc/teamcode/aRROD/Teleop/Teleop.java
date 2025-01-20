package org.firstinspires.ftc.teamcode.aRROD.Teleop;


import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.HEADING_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.HEADING_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.HEADING_P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.aRROD.utils.*;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers.PinpointLocalizer;



@TeleOp(name = "Static Heading Teleop", group = "aRRod")
public class Teleop extends LinearOpMode {

    double Kp = HEADING_P;
    double Kd = HEADING_D;
    double Ki = HEADING_I;
    double integralSum=0;
    boolean holdPos = false;
    public Pose whereToHold = new Pose();

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private Follower follower;



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor_ch_0");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor_ch_1");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor_ch_2");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motor_ch_3");
        follower = new Follower(hardwareMap);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        PinpointLocalizer drive = new PinpointLocalizer(hardwareMap);
        double refrenceAngle= 0;
        double timesCircled = 0;
        drive.resetIMU();



        waitForStart();

        while(opModeIsActive()) {
            follower.update();
            drive.update();
            refrenceAngle = -gamepad1.right_stick_x * .06 + refrenceAngle;


            timesCircled = (int) (refrenceAngle / (2 * Math.PI));
            refrenceAngle = refrenceAngle - (timesCircled * Math.PI * 2);
//            telemetry.addData("Current IMU Angle: ", drive.getPose().getHeading());
//            telemetry.addData("Target IMU Angle: ", refrenceAngle);
//            telemetry.addData("Times circled: ", timesCircled);
//            telemetry.addData("X pos: ", drive.getPose().getX());
//            telemetry.addData("Y pos: ", drive.getPose().getY());
            telemetry.addData("hold Pos: ", holdPos);
            telemetry.addData("where to hold: ", whereToHold);
            telemetry.addData("stick vals: ", Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_y));
            telemetry.update();

            double power = PIDControl(refrenceAngle, drive.getPose().getHeading());


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                drive.resetIMU();
            }


            double botHeading = drive.getPose().getHeading();
//            if (gamepad1.dpad_left) {
//                refrenceAngle = Math.toRadians(90);
//            }
//            if (gamepad1.dpad_right) {
//                refrenceAngle = Math.toRadians(270);
//            }


            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX), 1 - power);
            if (denominator > 1) {
                denominator = (1 / denominator) * 1 - power;
            }
            double frontLeftPower = ((rotY + rotX) * denominator) - power;
            double backLeftPower = ((rotY - rotX) * denominator) - power;
            double frontRightPower = ((rotY - rotX) * denominator) + power;
            double backRightPower = ((rotY + rotX) * denominator) + power;
//
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);



        }


    }

    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        telemetry.addData("Error: ", error);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }


}


