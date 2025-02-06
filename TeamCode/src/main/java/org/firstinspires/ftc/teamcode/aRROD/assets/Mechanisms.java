package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Mechanisms {
    public static DcMotorEx extendo_motor;
    public static ServoImplEx armL;
    public static ServoImplEx armR;
    public static ServoImplEx clawL;
    public static ServoImplEx clawR;
    public static DcMotor liftL;
    public static DcMotor liftR;

    //turret, arm, swivel
    public static ServoImplEx turret;
    public static ServoImplEx swivel;
    public static ServoImplEx extendo_arm;





    public static double oldTarget = 0;

    public static PIDController extendo_controller;
    public static PIDController lift_controller;
    public static int OFFSET = 0;
    public static boolean atPosition = false;
    public static Timer timer;
    public static Follower follower1;

    public Mechanisms(final HardwareMap hardwareMap, Follower follower){
        liftL = hardwareMap.dcMotor.get("motor_eh_1");
        liftR = hardwareMap.dcMotor.get("motor_ch_1");
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        follower1 = follower;

        extendo_motor = hardwareMap.get(DcMotorEx.class,"motor_ch_3");
        armL = hardwareMap.get(ServoImplEx.class, "servo_ch_1");
        armL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //armR = hardwareMap.get(Servo.class, "servo_ch_1");
        clawL = hardwareMap.get(ServoImplEx.class, "servo_ch_0");
        clawL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //clawR = hardwareMap.get(ServoImplEx.class, "location");
       //armR.setDirection(Servo.Direction.REVERSE);


        extendo_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret = hardwareMap.get(ServoImplEx.class, "servo_eh_0");
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendo_arm = hardwareMap.get(ServoImplEx.class, "servo_eh_2");
        extendo_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swivel = hardwareMap.get(ServoImplEx.class, "servo_eh_1");
        swivel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        timer = new Timer();
        extendo_controller = new PIDController(extendo_P,extendo_I,extendo_D);
        lift_controller = new PIDController(lift_P,lift_I,lift_D);
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


        int armPos = liftL.getCurrentPosition();
        double pid = lift_controller.calculate(armPos, lift_TARGET);
        double ff = Math.cos(Math.toRadians(lift_TARGET / LIFT_TICKS_IN_DEGREES)) * lift_F;
        double power = pid + ff;


        liftL.setPower(power);
        liftR.setPower(power);

    }


    public void armScore(){

        armL.setPosition(lift_arm_specimen_score);
        //armR.setPosition(lift_arm_specimen_score);
    }

    public void armIntake(){
        armL.setPosition(lift_arm_wall_intake);
        //armR.setPosition(lift_arm_wall_intake);
    }

    public void clawOpen(){
        clawL.setPosition(claw_wide_open);
        //clawR.setPosition(claw_wide_open);
    }
    public void clawClosed(){
        clawL.setPosition(claw_closed);
        //clawR.setPosition(claw_closed);
    }
    public void keepExtendoIn(){
        extendo_motor.setPower(-0.1);
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
    public void liftIntermediate(){
        lift_TARGET = (lift_pos_specimen * 1 / 2);
    }
    public boolean liftCloseEnough(){
        /*the pid will prolly mess up at some point but if we're within 75 ticks
        (and we'll change that to make it better) then we return true which means the lift
        is close enough to the target position and we can move on (thatll make sense later)
         */
        return liftL.getCurrentPosition() <= lift_TARGET + 55 && liftL.getCurrentPosition() >= lift_TARGET - 55;
    }
    public void servo_intake (){
        swivel.setPosition(nanoTape_intake);
        turret.setPosition(turret_intake);
        extendo_arm.setPosition(extendo_arm_pickup);
    }
    public void extendo_arm_camera_pos(){
        extendo_arm.setPosition(extendo_arm_camera_pos);
    }
    public void extendo_arm_prep_pos(){
        extendo_arm.setPosition(extendo_arm_prep_pos);
    }
    public void tape_swivel(double value){
        swivel.setPosition(swivel.getPosition()+0.007*value);
    }

    public void turret_swivel(double value){
        turret.setPosition(turret.getPosition()+0.007*value);
    }
    public void extendo_out(double value){
        extendo_TARGET += value*2;
        if(extendo_TARGET > 400){
            extendo_TARGET = 400;
        }
    }
    public void extendo_in_manual(double value){
        extendo_TARGET-=value*2;

    }
    public void extendo_transfer(){
        extendo_TARGET = 165;
    }
    public void extendo_arm_intake(){
        extendo_arm.setPosition(extendo_arm_pickup);
    }
    public void extendo_intake_transfer(){
        swivel.setPosition(nanoTape_transfer);
        turret.setPosition(turret_transfer);
        extendo_arm.setPosition(extendo_arm_transfer_pos);
    }

    public double getExtendoVoltage(){
        return extendo_motor.getCurrent(CurrentUnit.AMPS);
    }
    public double getExtendoTarget(){
        return extendo_motor.getCurrentPosition();
    }
    public void lift_specimen(){
        lift_TARGET = lift_pos_specimen;
    }
    public void arm_transfer(){
        armL.setPosition(lift_arm_transfer);
    }
    public double getLiftPos(){
        return liftL.getCurrentPosition();
    }
    public void prepTransfer1(){
        clawOpen();
        armL.setPosition(lift_arm_transfer);
        swivel.setPosition(0.5);
        turret.setPosition(0.55);
        lift_TARGET = lift_pos_rest+33;
        clawOpen();




    }
    public void prepTransfer2(){
        extendo_arm.setPosition(extendo_arm_transfer_pos);

    }
    public void resetMotors(){

        extendo_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendo_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void retryPrepArm(){
        extendo_arm.setPosition(extnedo_arm_retry_prep);
    }
    public boolean endOfPath(){
        return !follower1.isBusy();
    }


    public void autoPosEnd(Pose pose){
        autoToTeleop = pose;
    }
}



