package org.firstinspires.ftc.teamcode.aRROD.Teleop;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@TeleOp
@Config
public class hangTest extends LinearOpMode {
    public CRServo rightHang;
    public CRServo leftHang;

    public static String portName = "servo_ch_1";
    public static double servoPower = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        rightHang = hardwareMap.get(CRServo.class,"servo_eh_5");
        leftHang = hardwareMap.get(CRServo.class,"servo_ch_5");
        leftHang.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            rightHang.setPower(servoPower);
            leftHang.setPower(servoPower);

        }
    }
}