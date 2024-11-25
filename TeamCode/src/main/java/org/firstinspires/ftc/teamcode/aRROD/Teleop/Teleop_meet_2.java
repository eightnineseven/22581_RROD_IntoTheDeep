package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.ARM_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armIntake;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armIntermediate;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armRest;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armScore;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armSub;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.armSubIntermediate;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.clawClosed;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.clawOpen;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pivotHolding;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pivotScoring;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name="TELEOP COMP 2")
@Config
public class Teleop_meet_2 extends LinearOpMode {
//    private PIDController controller;
    public static double target = 0;
    public static double p = ARM_P, i = ARM_I, d = ARM_D;
    public static double offset = 0.1;
    public static double f = ARM_F;
    public static double clawPos=0;
    public static double pivotPos=0;
    private final double ticks_in_degree = 28*5.24*5.24*5.24/360;
    private PIDController controller;
    public static boolean cycleIntermediate = false;
    public static double oldTarget = 0;
    public static boolean STOP = false;
    public Timer timer;
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo clawR = hardwareMap.get(Servo.class, "servo_ch_1");
        Servo clawL = hardwareMap.get(Servo.class, "servo_ch_2");
        Servo clawPivot = hardwareMap.get(Servo.class, "servo_ch_0");
        clawR.setDirection(Servo.Direction.REVERSE);
        DcMotor arm_motor = hardwareMap.get(DcMotor.class, "motor_eh_0");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new PIDController(p,i,d);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        timer = new Timer();
        GoBildaPinpointDriver imu = hardwareMap.get(GoBildaPinpointDriver.class,"imu 1");
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();



        waitForStart();
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.resetTimer();
        while(opModeIsActive()&&!isStopRequested()) {


            imu.update();

            controller.setPID(p,i,d);

            int armPos = arm_motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            //this formula works most accurately i've found btw
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double    power = pid + ff;

            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            telemetry.addData("Target: ", target);
            telemetry.addData("Position: ", arm_motor.getCurrentPosition());
            telemetry.update();
            if(STOP){
                if(gamepad2.left_trigger>0.1){
                    arm_motor.setPower(gamepad2.left_trigger);
                } else if(gamepad2.right_trigger>0.1){
                    arm_motor.setPower(-gamepad2.right_trigger);
                } else arm_motor.setPower(0.1);
            } else {
                arm_motor.setPower(power);
            }
            if(gamepad2.b){
                target=(int)armScore;
            }
            if(gamepad2.x){
                target=(int)armRest;

                /////
            }
            if(gamepad2.a){
                target=(int)armIntake;
            }
            if(gamepad2.y){
                target=(int)armIntermediate;
            }
            if(gamepad2.dpad_left&&!cycleIntermediate){
                target = (int)armSub;
                cycleIntermediate = true;
            } else if(gamepad2.dpad_left&&cycleIntermediate){
                clawPivot.setPosition(pivotHolding-offset);
                target = (int)armSubIntermediate;
                cycleIntermediate = false;
            }
            if(currentGamepad2.dpad_left&&!previousGamepad2.dpad_left){
                cycleIntermediate =!cycleIntermediate;
            }
            if(gamepad2.dpad_up){
                arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad2.right_bumper){
                clawPivot.setPosition(pivotHolding);
            }
            if(gamepad2.left_bumper){
                clawPivot.setPosition(pivotScoring);
            }
            if(gamepad2.left_stick_button){
                clawL.setPosition(clawClosed);
                clawR.setPosition(clawClosed);
            }
            if(gamepad2.right_stick_button){
                clawL.setPosition(clawOpen);
                clawR.setPosition(clawOpen);
            }
            if(gamepad2.dpad_right){
                STOP=true;
            }
            if(gamepad2.dpad_down){
                STOP=false;
            }
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.left_stick_button) {
                imu.recalibrateIMU();
            }

            double botHeading = imu.getHeading();

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

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }

    }
}
