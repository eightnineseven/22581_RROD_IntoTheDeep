package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class hangTest extends LinearOpMode {
    public static double hangPower = 0.4;
    public DcMotor arm_motor;
    @Override
    public void runOpMode() throws InterruptedException {
        arm_motor = hardwareMap.dcMotor.get("motor_eh_0");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            if(gamepad1.left_trigger>0.1){
                arm_motor.setPower(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger>0.1){
                arm_motor.setPower(-gamepad1.right_trigger);
            } else{
                arm_motor.setPower(0);
            }
            telemetry.addData("motor power: ",arm_motor.getPower());
            telemetry.update();
        }

    }
}
