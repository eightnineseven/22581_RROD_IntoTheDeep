package org.firstinspires.ftc.teamcode.RROD.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RROD.util.GLOBALS;
import org.firstinspires.ftc.teamcode.RROD.visionPipelines.BlueSampleOrientationAnalysisPipeline;
import org.firstinspires.ftc.teamcode.RROD.visionPipelines.RedSampleOrientationAnalysisPipeline;

import com.arcrobotics.ftclib.controller.PIDController;

@TeleOp
@Config
public class Teleop extends LinearOpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public GLOBALS globals;
    public BlueSampleOrientationAnalysisPipeline blueVision = new BlueSampleOrientationAnalysisPipeline();
    public RedSampleOrientationAnalysisPipeline redVision = new RedSampleOrientationAnalysisPipeline();
    public static double target = 0;

    private final double ticks_in_degree = 537.7/180;

    public static int prep_state = 0;
    public static int high_chamber_pos = 0;
    public static int plow_pos = 0;
    public static int resting_pos = 0;
    public boolean prep_state_cycle = true;

    private DcMotor arm_motor;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("motor_ch_0");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("motor_ch_1");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("motor_eh_0");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("motor_eh_1");

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.dcMotor.get("motor_eh_2");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


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
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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


            if(gamepad1.a && prep_state_cycle){
                target = prep_state;
                prep_state_cycle = false;
            } else if(gamepad1.a){
                target = prep_state - 3;
                prep_state_cycle = true;
            }
            if(gamepad1.y){
                target = high_chamber_pos;
            }
            if(gamepad1.b){
            target = plow_pos;
            }
            if(gamepad1.x){
                target = resting_pos;
            }

            //TODO: add camera to opmode
            if(globals.getAllianceColor()== GLOBALS.ALLIANCE.BLUE){
                telemetry.addData("X diff: ", blueVision.getBlueXDifference());
                telemetry.addData("Y diff: ", blueVision.getBlueYDifference());
                telemetry.update();
            } else{
                telemetry.addData("X diff: ", redVision.getRedXDifference());
                telemetry.addData("Y diff: ", redVision.getRedYDifference());
                telemetry.update();

            }



            
            
        }
    }
}
