package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.EXTENDO_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;

@TeleOp
@Config
public class liftPIDTuner extends LinearOpMode {
    public static double extendo_TARGET=0;
    public static boolean extendo_toggle = true;
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad=new Gamepad();
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor motor = hardwareMap.dcMotor.get("motor_ch_3");
        PIDController extendo_controller = new PIDController(extendo_P, extendo_I, extendo_D);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while(opModeIsActive() && !isStopRequested()){
            extendo_controller.setPID(extendo_P, extendo_I, extendo_D);
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            if(currentGamepad.dpad_up &&!previousGamepad.dpad_up){
                extendo_toggle =! extendo_toggle;
            }
            if (extendo_toggle) {
                extendo_TARGET = 250;
            } else{
                extendo_TARGET = 0;
            }

            //random calculating bullshit that idk how it actually works but it does
            int armPos = motor.getCurrentPosition();
            double pid = extendo_controller.calculate(armPos, extendo_TARGET);
            double ff = Math.cos(Math.toRadians(extendo_TARGET / EXTENDO_TICKS_IN_DEGREES)) * extendo_F;
            double power = pid + ff;

            //the important part here
            motor.setPower(power);
            telemetry.addData("target: ", extendo_TARGET);
            telemetry.addData("position: ", motor.getCurrentPosition());
            telemetry.update();


        }

    }
}
