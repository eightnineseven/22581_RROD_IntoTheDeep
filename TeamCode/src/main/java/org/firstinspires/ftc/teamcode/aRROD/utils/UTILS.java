package org.firstinspires.ftc.teamcode.aRROD.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class UTILS {
    public enum teamColor{
        RED,
        BLUE
    }
    public static teamColor TEAM_COLOR= teamColor.BLUE;






 //****************************************************
 //*************************************************

    //SERVO AND ARM POSITIONS:





    public static double HEADING_P=2;
    public static double HEADING_I=0;
    public static double HEADING_D=0;
   public static double extendo_P=0.0132;
   public static double extendo_I=0.0001;
   public static double extendo_D=0.0005;
   public static double extendo_F=0.01;
   public static double lift_P=0.016;
   public static double lift_I = 0.3;
   public static double lift_D = 0.0005;
   public static double lift_F=0.12;
   public static double lift_arm_specimen_score =  0.37;
   public static double lift_arm_wall_intake = 0.98;
   
   
   public static double claw_wide_open = 0.2;
   public static double claw_slight_open = 0.1;
   public static double claw_closed = 0;
   public static double turret_swivel_0 = 0.2;
   public static double turret_swivel_180 = 0.9;
   public static double tape_swivel_0 = 0.2;
   public static double tape_swivel_180=0.9;
   public static long SERVO_WAIT_TIME = 150;
    public static double LIFT_TICKS_IN_DEGREES= (double) 145 /360;
    public static double EXTENDO_TICKS_IN_DEGREES = 100;
    public static double lift_pos_specimen=400;
    public static double lift_pos_rest = 0;
    //
    // intake arm positions
    public static double lift_arm_bucket_score = 0.6;
    public static double lift_arm_transfer = 0.06;
    public static double extendo_arm_pickup = 0;
    public static double lift_pos_bucket = 0;
    public static double extendo_out = 0;
    public static double turret_transfer = 0.5;
    public static double turret_intake = 0;
    public static double nanoTape_transfer = 0.5;
    public static double nanoTape_intake = 0;
    public static double LENGTH_OF_BEAM = 4;
    public static double ROBOT_OFFSET = 0;
    public static double PIXELS_PER_INCH = 96;
    public static double extendo_arm_camera_pos = 0.6;
    public static double extendo_arm_prep_pos  = 0.2;
    public static double extnedo_arm_retry_prep = 0.15;
    public static double extendo_TARGET = 0;
    public static double lift_TARGET = 0;
    public static double extendo_arm_transfer_pos=0.92;
    public static double lift_transfer_pos = 100;
    public static double lift_arm_transfer_pos = 0;
    public static Pose autoToTeleop = new Pose();
}





    

