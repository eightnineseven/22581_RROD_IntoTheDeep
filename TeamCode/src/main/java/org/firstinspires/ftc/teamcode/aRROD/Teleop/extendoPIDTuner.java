package org.firstinspires.ftc.teamcode.aRROD.Teleop;



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

        while(opModeIsActive() && !isStopRequested()){
            mechs.extendoUpdate();
        }

    }
}
