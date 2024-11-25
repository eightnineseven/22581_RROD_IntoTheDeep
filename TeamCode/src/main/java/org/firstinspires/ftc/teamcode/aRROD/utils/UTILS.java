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





    public static int REST = 0;
    public static int READY_FOR_INTAKE = 1;
    public static int INTAKE_PIECE = 2;
    public static int OUTTAKE = 3;


    public static double HEADING_P=2;
    public static double HEADING_I=0;
    public static double HEADING_D=0;
    public static double clawClosed = 0;
    public static double clawOpen = 0.2;
    public static double pivotScoring = 0;
    public static double pivotHolding = 0.3;
    public static double pivotSub = 0.15;
    public static double armIntake =230;
    public static double armRest = 0;
    public static double armIntermediate = 530;
    public static double armScore = 960;
    public static double armSub = 2200;
    public static double armSubIntermediate = 1900;
    public static double ARM_P=0.0067;
    public static double ARM_I=0;
    public static double ARM_D=0.0004;
    public static double ARM_F=0.1;
    public static double ARM_TICKS_IN_DEGREES=28*5.24*5.24*5.24/360;
    }




    

