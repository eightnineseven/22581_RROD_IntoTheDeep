package org.firstinspires.ftc.teamcode.aRROD.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class UTILS {
    public enum teamColor{
        RED,
        BLUE
    }
    public static teamColor TEAM_COLOR= teamColor.BLUE;
    public static double startX = 7.6;
    public static double startY = 64;

    public static double preloadPlaceX = 33;
    public static double preloadPlaceY = 62.7;



    //-------------------------------------------------
    public static double sample1X = 15;
    public static double sample1Y = 20;

    public static double firstSample1WeightX = 0;
    public static double firstSample1WeightY = 53;

    public static double secondSample1WeightX = 58;
    public static double secondSample1WeightY=34;

    public static double placeSecondSpecX = 35.4;
    public static double placeSecondSpecY = 65;

//-------------------------------------------------




    public static double sample2X = 15.5;
    public static double sample2Y = 14;

    public static double firstSample2WeightX = 0;
    public static double firstSample2WeightY = 50;

    public static double secondSample2WeightX = 58;
    public static double secondSample2WeightY=24;

    public static double placeThirdSpecX = 33.6;
    public static double placeThirdSpecY = 66;



//-------------------------------------------------

    public static double sample3X = 14.5;
    public static double sample3Y = 14;

    public static double firstSample3WeightX = 0;
    public static double firstSample3WeightY = 47;

    public static double secondSample3WeightX = 58;
    public static double secondSample3WeightY=10;

    public static double placeFourthSpecX = 34.1;
    public static double placeFourthSpecY = 68;

   //-------------------------------------------------





    public static double pickupFifthSpecX = 3;
    public static double pickupFifthSpecY=6;

    public static double placeFifthSpecX = 33;
    public static double placeFifthSpecY = 66;


    //-------------------------------------------------

    public static double parkX= 7;
    public static double parkY = 24;

    public static double prePlaceX = 35.65;
    public static double prePlaceY = placeFifthSpecY;
    public static double postPlaceX = 32;
    public static double postPlaceY = placeFifthSpecY;






 //****************************************************
 //*************************************************

    //SERVO AND ARM POSITIONS:





    public static double HEADING_P=2;
    public static double HEADING_I=0;
    public static double HEADING_D=0;
   public static double extendo_P=0;
   public static double extendo_I=0;
   public static double extendo_D=0;
   public static double extendo_F=0;
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
    public static double EXTENDO_TICKS_IN_DEGREES = 0;
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
}





    

