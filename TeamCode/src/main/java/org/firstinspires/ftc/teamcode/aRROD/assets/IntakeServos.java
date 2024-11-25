package org.firstinspires.ftc.teamcode.aRROD.assets;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServos {
    public static double OPEN = clawOpen;
    public static double CLOSED = clawClosed;
    public static double HOLDING = pivotHolding;
    public static double SCORING = pivotScoring;
    public static Servo clawL;
    public static Servo clawR;
    public static Servo pivot;
    public IntakeServos(final HardwareMap hardwareMap){
         clawR = hardwareMap.get(Servo.class, "servo_ch_1");
         clawL = hardwareMap.get(Servo.class, "servo_ch_2");
         pivot = hardwareMap.get(Servo.class, "servo_ch_0");
        clawR.setDirection(Servo.Direction.REVERSE);
    }
    public void closeClaw(){
        clawR.setPosition(CLOSED);
        clawL.setPosition(CLOSED);
    }
    public void flipIn(){
        pivot.setPosition(HOLDING);
    }
    public void openClaw(){
        clawR.setPosition(OPEN);
        clawL.setPosition(OPEN);
    }
    public void flipOut(){
        pivot.setPosition(SCORING);
    }


}
