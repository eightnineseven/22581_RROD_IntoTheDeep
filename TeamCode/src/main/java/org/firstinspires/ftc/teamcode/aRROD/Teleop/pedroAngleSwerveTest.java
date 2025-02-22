package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveFollower;
@Config
public class pedroAngleSwerveTest extends LinearOpMode {
    public static double frontLeft = 0;
    public static double backLeft = 0;
    public static double frontRight = 0;
    public static double backRight = 0;
    public static boolean goForward = false;
    public static boolean leftStrafe = false;
    public static boolean rightStrafe = false;
    public static boolean goBackward = false;
    public static boolean turnLeft = false;
    public static boolean turnRight = false;
    public static double[] forwardPowers = {1,1,1,1};
    public static double[] backwardsPowers = {-1,-1,-1,-1};
    public static double[] strafeLeftPowers = {-1,1,1,-1};
    public static double[] strafeRightPowers = {1,-1,-1,1};
    public static double[] turnLeftPowers = {-1,-1,1,1};
    public static double[] turnRightPowers = {1,1,-1,-1};
    double[] dashboardControlled = {frontLeft,frontRight,frontRight,backRight};
    double[] finalPowers;


    @Override
    public void runOpMode() throws InterruptedException {
        SwerveFollower follower = new SwerveFollower(hardwareMap);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            if(goForward){
                finalPowers = forwardPowers;
            } else if(goBackward){
                finalPowers = backwardsPowers;
            } else if(leftStrafe){
                finalPowers = strafeLeftPowers;
            } else if(rightStrafe){
                finalPowers = strafeRightPowers;
            } else if(turnLeft){
                finalPowers = turnLeftPowers;
            } else if(turnRight){
                finalPowers = turnRightPowers;
            } else{
                finalPowers = dashboardControlled;
            }
            follower.update();
            follower.setServoAngle(finalPowers);


        }

    }
}
