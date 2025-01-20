package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class omgSloth extends LinearOpMode {
    public static double servoPos = 0.4;
    public Servo servo1;
    public Servo servo2;
    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(Servo.class,"servo_ch_0");
        servo2 = hardwareMap.get(Servo.class,"servo_ch_1");
        servo1.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            servo1.setPosition(servoPos);
            servo2.setPosition(servoPos);
            //new line
        }

    }
}
