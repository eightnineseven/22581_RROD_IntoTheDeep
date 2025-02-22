package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms.extendo_arm;
import static org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms.turret;
import static org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms.swivel;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.EXTENDO_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.LIFT_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_TARGET;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.lift_TARGET;

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
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;

@TeleOp
@Config
public class outreachslides extends LinearOpMode {
    public static double turretpos = 0.5;
    public static double extendoarmpos = 0.5;
    public static double swivelpos = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        PIDController lift_controller = new PIDController(lift_P,lift_I,lift_D);
        PIDController extendo_controller = new PIDController(extendo_P, extendo_I, extendo_D);
        extendo_controller = new PIDController(extendo_P,extendo_I,extendo_D);
        DcMotor liftL = hardwareMap.dcMotor.get("motor_eh_1");
        DcMotor liftR = hardwareMap.dcMotor.get("motor_ch_1");
        DcMotor extendo_motor = hardwareMap.dcMotor.get("motor_ch_3");
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = hardwareMap.get(ServoImplEx.class, "servo_eh_0");
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        extendo_arm = hardwareMap.get(ServoImplEx.class, "servo_eh_2");
        extendo_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swivel = hardwareMap.get(ServoImplEx.class, "servo_eh_1");
        swivel.setPwmRange(new PwmControl.PwmRange(500, 2500));
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            lift_controller.setPID(lift_P, lift_I, lift_D);
            int armPos = liftL.getCurrentPosition();
            double pid = lift_controller.calculate(armPos, lift_TARGET);
            double ff =(lift_TARGET / LIFT_TICKS_IN_DEGREES) * lift_F;
            double power = pid + ff;
            int extendoPos = extendo_motor.getCurrentPosition();
            double epid = extendo_controller.calculate(extendoPos, extendo_TARGET);
            double eff =(extendo_TARGET / EXTENDO_TICKS_IN_DEGREES) * 0.01;
            double epower = epid + eff;
            extendo_motor.setPower(epower);


            liftL.setPower(power);
            liftR.setPower(power);
            turret.setPosition(turretpos);
            extendo_arm.setPosition(extendoarmpos);
            swivel.setPosition(swivelpos);

            telemetry.addData("lift target: ", lift_TARGET);
            telemetry.addData("lift pos: ", liftL.getCurrentPosition());
            telemetry.update();

        }

    }
}
