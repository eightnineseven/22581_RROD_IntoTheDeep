package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.LIFT_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_TARGET;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_arm_specimen_score;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;

@TeleOp
@Config
public class liftPIDTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PIDController lift_controller = new PIDController(lift_P,lift_I,lift_D);

        DcMotor liftL = hardwareMap.dcMotor.get("motor_eh_1");
        DcMotor liftR = hardwareMap.dcMotor.get("motor_ch_1");
        ServoImplEx armL = hardwareMap.get(ServoImplEx.class, "servo_ch_1");
        armL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            lift_controller.setPID(lift_P, lift_I, lift_D);
            int armPos = liftL.getCurrentPosition();
            double pid = lift_controller.calculate(armPos, lift_TARGET);
            double ff =Math.cos(lift_TARGET / LIFT_TICKS_IN_DEGREES) * lift_F;
            double power = pid + ff;


            liftL.setPower(power);
            liftR.setPower(power);

            telemetry.addData("lift target: ", lift_TARGET);
            telemetry.addData("lift pos: ", liftL.getCurrentPosition());
            telemetry.update();

        }

    }
}



