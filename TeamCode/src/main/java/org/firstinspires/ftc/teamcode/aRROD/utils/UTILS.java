package org.firstinspires.ftc.teamcode.aRROD.utils;

import com.acmerobotics.dashboard.config.Config;

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
   public static double extendo_I=0;
   public static double extendo_D=0.0005;
   public static double extendo_F=0.01;
   public static double lift_P=0;
   public static double lift_I = 0;
   public static double lift_D = 0;
   public static double lift_F=0;
   public static double lift_arm_specimen_score = 0.7;
   public static double lift_arm_wall_intake = 0.2;
   
   
   public static double claw_wide_open = 0.2;
   public static double claw_slight_open = 0.1;
   public static double claw_closed = 0;
   public static double turret_swivel_0 = 0;
   public static double turret_swivel_180 = 0;
   public static double tape_swivel_0 = 0;
   public static double tape_swivel_180=0;
   public static long SERVO_WAIT_TIME = 150;
    public static double LIFT_TICKS_IN_DEGREES= (double) 145 /360;
    public static double EXTENDO_TICKS_IN_DEGREES = 100;
    public static double lift_pos_specimen=0;
    public static double lift_pos_rest = 0;
    //
    // intake arm positions
    public static double lift_arm_bucket_score = 0;
    public static double lift_arm_transfer = 0;
    public static double extendo_arm_pickup = 0;
    public static double lift_pos_bucket = 0;
    public static double extendo_out = 0;
    public static double turret_transfer = 0;
    public static double turret_intake = 0;
    public static double nanoTape_transfer = 0;
    public static double nanoTape_intake = 0;
    public static double LENGTH_OF_BEAM = 4;
    public static double ROBOT_OFFSET = 0;
    public static double PIXELS_PER_INCH = 96;
    public static double extendo_arm_camera_pos = 0.6;
}





    

