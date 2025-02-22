package org.firstinspires.ftc.teamcode.aRROD.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.SwerveFollower;

public class swerveTeleopTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveFollower follower = new SwerveFollower(hardwareMap);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            follower.update();
            follower.setServoAngle(gamepad1.right_stick_x + gamepad1.right_stick_y);
            follower.setMotorPower(gamepad1.left_stick_y);

        }
    }

}
