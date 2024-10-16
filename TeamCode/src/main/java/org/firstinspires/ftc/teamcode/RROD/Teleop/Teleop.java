package org.firstinspires.ftc.teamcode.RROD.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.RROD.util.GLOBALS;

@TeleOp
@Config
public class Teleop extends LinearOpMode {
    private PIDController controller;
    GoBildaPinpointDriverRR odo;
    public static double p = 0.007, i = 0, d = 0.00085;
    public static double f = 0.36;
    public GLOBALS globals;

//    public SampleOrientationAnalysisPipeline sampleOrientation = new SampleOrientationAnalysisPipeline();
    public static double target = 0;
    public static double clawLPos = 0;
    public static double clawRPos = 0;

    private final double ticks_in_degree = 537.7/180;

    public static int prep_state = 270;
    public static int high_chamber_pos = 180;

    public static int resting_pos = 10;
    public boolean prep_state_cycle = true;
    public double servoPos = 0;


    private DcMotor arm_motor;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor_ch_0");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor_ch_1");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor_ch_2");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motor_ch_3");

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.dcMotor.get("motor_eh_0");

        Servo clawPivot = hardwareMap.servo.get("servo_ch_0");
        Servo clawR = hardwareMap.servo.get("servo_ch_1");
        Servo clawL = hardwareMap.servo.get("servo_ch_2");
        clawR.setDirection(Servo.Direction.REVERSE);

        

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map



        //PID:
        

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controller.setPID(p,i,d);
            int armPos = arm_motor.getCurrentPosition();
            double pid = controller.calculate(armPos,target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree))*f;
            double power = pid +ff;
           arm_motor.setPower(power);
            arm_motor.setPower(0);

            telemetry.addData("pos: ", armPos);
            telemetry.addData("target: ", target);
            telemetry.update();






            
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                odo.recalibrateIMU();
            }

            double botHeading = Math.toRadians(odo.getHeading());

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if(gamepad1.circle){
                target = prep_state;

                clawPivot.setPosition(0.7);

            } else if(gamepad1.triangle){
                clawPivot.setPosition(0.5);

            }
            if(gamepad1.square){

                target = high_chamber_pos;
                clawPivot.setPosition(0.3);

            }

            if(gamepad1.x){
                target = resting_pos;
                clawPivot.setPosition(0.45);

            }
            if(gamepad1.dpad_up){
                clawL.setPosition(0);
                clawR.setPosition(0);
            }
            if(gamepad1.dpad_down){
                clawL.setPosition(0.7);
                clawR.setPosition(0.7);
            }
            if(gamepad1.dpad_left){
                clawL.setPosition(0.15);
                clawR.setPosition(0.15);
            }








            //TODO: add camera to opmode

//                telemetry.addData("X diff: ", sampleOrientation.getRedXDifference());
//                telemetry.addData("Y diff: ", sampleOrientation.getRedYDifference());
//                telemetry.addData("Angle: ", sampleOrientation.getAngle());
//                telemetry.update();





            
            
        }
    }
}
