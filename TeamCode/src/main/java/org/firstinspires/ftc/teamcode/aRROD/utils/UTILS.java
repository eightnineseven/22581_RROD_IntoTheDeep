package org.firstinspires.ftc.teamcode.aRROD.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class UTILS {
    public enum teamColor{
        RED,
        BLUE
    }
    public static teamColor TEAM_COLOR= teamColor.RED;






 //****************************************************
 //*************************************************

    //SERVO AND ARM POSITIONS:





    public static double HEADING_P=2;
    public static double HEADING_I=0;
    public static double HEADING_D=0;
   public static double extendo_P=0.0002;
   public static double extendo_I=0.02;
   public static double extendo_D=0.00002;
   public static double extendo_F=0.0;
   public static double extendo_MAX = 29000;
   public static double extendo_MIN = 0;
   public static double extendo_transfer = 12000;
   public static double lift_P=0.016;
   public static double lift_I = 0.3;
   public static double lift_D = 0.0005;
   public static double lift_F=0.12;
   public static double lift_arm_specimen_score =  0.51;
   public static double lift_arm_wall_intake = 0.98;
   
   
   public static double claw_wide_open = 0.14;
   public static double claw_slight_open = 0.1;
   public static double claw_closed = 0;
   public static double turret_swivel_0 = 0;
   public static double turret_swivel_180 = 1;
   public static double tape_swivel_0 = 0.2;
   public static double tape_swivel_180=0.9;
   public static long SERVO_WAIT_TIME = 150;
    public static double LIFT_TICKS_IN_DEGREES= 0.4027777777777777;
    public static double EXTENDO_TICKS_IN_DEGREES = 22.75555555555;
    public static double lift_pos_spec_prep =50;
    public static double lift_pos_spec_score = 220;
    public static double lift_pos_rest = 0;
    //
    // intake arm positions
    public static double lift_arm_bucket_score = 0.6;
    public static double lift_arm_transfer = 0.13;
    public static double extendo_arm_pickup = 0.08;
    public static double lift_pos_bucket = 0;
    public static double extendo_out = 0;
    public static double turret_transfer = .55;
    public static double turret_intake = 0;
    //NT TRANSFER DOES NOTHING
    public static double nanoTape_transfer = 0.55;
    public static double nanoTape_intake = 0.25;
    public static double LENGTH_OF_BEAM = 4;
    public static double ROBOT_OFFSET = 0;
    public static double PIXELS_PER_INCH = 96;
    public static double extendo_arm_camera_pos = 0.25;
    public static double extendo_arm_prep_pos  = .14;
    public static double extnedo_arm_retry_prep = 0.15;
    public static double extendo_TARGET = 0;
    public static double lift_TARGET = 0;
    public static double extendo_arm_transfer_pos=0.89;
    public static double lift_transfer_pos = 100;
    public static double lift_arm_transfer_pos = 0;
    public static Pose autoToTeleop = new Pose();
    public static double length_of_beam_score = 5.45;
    public static double length_of_beam_cam = 5;
    public static double full_exension_ticks = 29000;
    public static double full_extension_inches = 14.173;
}





    

