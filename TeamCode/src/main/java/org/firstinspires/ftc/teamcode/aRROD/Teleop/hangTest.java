package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
@Config
public class hangTest extends LinearOpMode {
    public static double servoPos = 0;
    public ServoImplEx servo1;
    public static String portName = "servo_ch_1";
    @Override
    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(ServoImplEx.class,portName);
        servo1.setPwmRange(new PwmControl.PwmRange(500,2500));
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            servo1.setPosition(servoPos);

            //new line
        }

    }
}
