package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class Mechanisms {
    public static DcMotor extendo_motor;
    public static Servo armL;
    public static Servo armR;
    public static Servo clawL;
    public static Servo clawR;
    public static DcMotor liftL;
    public static DcMotor liftR;

    //turret, arm, swivel
    public static Servo turret;
    public static Servo swivel;
    public static Servo armBucket_Intake;



    public static double extendo_TARGET = 0;
    public static double lift_TARGET=0;
    public static double oldTarget = 0;

    public static PIDController extendo_controller;
    public static PIDController lift_controller;
    public static int OFFSET = 0;
    public static boolean atPosition = false;
    public static Timer timer;

    public Mechanisms(final HardwareMap hardwareMap){
        extendo_motor = hardwareMap.dcMotor.get("location");
        armL = hardwareMap.get(Servo.class, "location");
        armR = hardwareMap.get(Servo.class, "location");
        clawL = hardwareMap.get(Servo.class, "location");
        clawR = hardwareMap.get(Servo.class, "location");
        armR.setDirection(Servo.Direction.REVERSE);
        extendo_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret = hardwareMap.get(Servo.class, "location");
        armBucket_Intake = hardwareMap.get(Servo.class, "location");
        swivel = hardwareMap.get(Servo.class, "location");
        timer = new Timer();
    }
    public void extendoUpdate() {
        extendo_controller.setPID(extendo_P, extendo_I, extendo_D);

        //random calculating bullshit that idk how it actually works but it does
        int armPos = extendo_motor.getCurrentPosition();
        double pid = extendo_controller.calculate(armPos, extendo_TARGET);
        double ff = Math.cos(Math.toRadians(extendo_TARGET / EXTENDO_TICKS_IN_DEGREES)) * extendo_F;
        double power = pid + ff;

        //the important part here
        extendo_motor.setPower(power);

    }
    public void liftUpdate(){
        //same as extendo but different motor
        lift_controller.setPID(lift_P, lift_I, lift_D);


        int armPos = extendo_motor.getCurrentPosition();
        double pid = extendo_controller.calculate(armPos, lift_TARGET);
        double ff = Math.cos(Math.toRadians(extendo_TARGET / EXTENDO_TICKS_IN_DEGREES)) * lift_F;
        double power = pid + ff;


        liftL.setPower(power);
        liftR.setPower(power);

    }


    public void armScore(){

        armL.setPosition(lift_arm_specimen_score);
        armR.setPosition(lift_arm_specimen_score);
    }

    public void armIntake(){
        armL.setPosition(lift_arm_wall_intake);
        armR.setPosition(lift_arm_wall_intake);
    }

    public void clawOpen(){
        clawL.setPosition(claw_wide_open);
        clawR.setPosition(claw_wide_open);
    }
    public void clawClosed(){
        clawL.setPosition(claw_closed);
        clawR.setPosition(claw_closed);
    }
    public void liftUp(){
        //sets the target value in the update method that i showed earlier
        lift_TARGET = lift_pos_specimen;
    }
    public void liftDown(){
        lift_TARGET = lift_pos_rest;
    }
    public void extendoClose(){
        extendo_TARGET = 0;
    }
    public boolean liftCloseEnough(){
        /*the pid will prolly mess up at some point but if we're within 75 ticks
        (and we'll change that to make it better) then we return true which means the lift
        is close enough to the target position and we can move on (thatll make sense later)
         */
        return liftL.getCurrentPosition() <= lift_TARGET + 75 && liftL.getCurrentPosition() >= lift_TARGET - 75;
    }
    public void servo_intake (){
        swivel.setPosition(nanoTape_intake);
        turret.setPosition(turret_intake);
        armBucket_Intake.setPosition(extendo_arm_pickup);
    }


    }


