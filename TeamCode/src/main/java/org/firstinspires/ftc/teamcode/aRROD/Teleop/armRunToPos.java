package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.pivotHolding;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class armRunToPos extends LinearOpMode {
        public static int position = 0;
        public static double power = 0.2;
        DcMotor arm_motor;
    @Override
    public void runOpMode() throws InterruptedException {
        arm_motor = hardwareMap.dcMotor.get("motor_eh_0");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        Servo clawPivot = hardwareMap.get(Servo.class, "servo_ch_0");
        clawPivot.setPosition(pivotHolding);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            if(gamepad1.dpad_up) {
                arm_motor.setTargetPosition(position);
                arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_motor.setPower(power);
            }
            if(gamepad1.dpad_down){
                arm_motor.setPower(0.1);
            }
            if(gamepad1.dpad_left){
                arm_motor.setPower(0);
            }
            telemetry.addData("Current arm pos: ", arm_motor.getCurrentPosition());
            telemetry.addData("target arm pos: ", position);
            telemetry.update();

        }

    }
}
