package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Arm {
    public static DcMotor arm_motor;
    public static double SCORING = armScore;
    public static double INTAKE = armIntake;
    public static double REST = armRest;
    public static double TARGET = 0;
    public static double oldTarget = 0;
    private final double ticks_in_degrees = 28*5.24*5.24*5.24/360;
    public static PIDController controller;
    public static int OFFSET = 0;
    public static boolean atPosition = false;
    public static Timer timer;

    public Arm(final HardwareMap hardwareMap){
        arm_motor = hardwareMap.dcMotor.get("motor_eh_0");
        arm_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDController(ARM_P,ARM_I,ARM_D);
        timer = new Timer();
    }
    public void armUpdate() {
        controller.setPID(ARM_P, ARM_I, ARM_D);


        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, TARGET);
        double ff = Math.cos(Math.toRadians(TARGET / ticks_in_degrees)) * ARM_F;
        double power = pid + ff;


        arm_motor.setPower(power);

    }

    public void outtake(){
        TARGET = SCORING;
        timer.resetTimer();
    }
    public void intake(){
        TARGET = INTAKE;
        timer.resetTimer();
    }
    public void rest(){
        TARGET = REST;
        timer.resetTimer();
    }
    public void resetArm(){
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void restartArm(){
        arm_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void intermediate(){
        TARGET = armIntermediate;
        timer.resetTimer();
    }

}
