package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_TARGET;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;

@TeleOp
@Config
public class extendoPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mechanisms mechs = new Mechanisms(hardwareMap);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            mechs.liftUpdate();
            telemetry.addData("lift pos: ", mechs.getLiftPos());
            telemetry.addData("lift target: ", lift_TARGET );
            telemetry.update();
        }

    }
}
