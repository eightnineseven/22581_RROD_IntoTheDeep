package org.firstinspires.ftc.teamcode.aRROD.Teleop;



import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.EXTENDO_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.LIFT_TICKS_IN_DEGREES;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_D;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_F;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_I;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_P;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_TARGET;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.full_exension_ticks;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.full_extension_inches;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.length_of_beam_cam;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.length_of_beam_score;
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
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Encoder;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.aRROD.assets.Mechanisms;
import org.firstinspires.ftc.teamcode.aRROD.assets.SampleOrientationAnalysisPipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Config
public class extendoPIDTuner extends LinearOpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    public static double tapePos = 0;
    public static double turretPos = 0;
    public static double testpos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        Follower follower = new Follower(hardwareMap);
        Mechanisms mechs = new Mechanisms(hardwareMap, follower);



        PIDController extendo_controller = new PIDController(extendo_P,extendo_I,extendo_D);
        DcMotor extendo_motor = hardwareMap.get(DcMotorEx.class,"motor_ch_3");
        DcMotor encoder = hardwareMap.get(DcMotor.class, "motor_eh_2");








        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        SampleOrientationAnalysisPipeline angleFinder = new SampleOrientationAnalysisPipeline(telemetry);
        VisionPortal visionPortal;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(angleFinder)
                .enableLiveView(true)
                .build();
        waitForStart();

        mechs.turretMove(0.7);
        while(opModeIsActive() && !isStopRequested()){
            visionPortal.resumeLiveView();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            extendo_controller.setPID(extendo_P, extendo_I, extendo_D);
            int armPos = -encoder.getCurrentPosition();
            double pid = extendo_controller.calculate(armPos, extendo_TARGET);
            double ff =(extendo_TARGET / EXTENDO_TICKS_IN_DEGREES) * extendo_F;
            double power = pid + ff;



            extendo_motor.setPower(power);



            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                mechs.extendo_arm_camera_pos();
            }
            double angleOfSample = Math.toRadians(angleFinder.getAngleOfSample());
            double true_samp_ang = angleOfSample + mechs.turret_pos();
            double x_offset = length_of_beam_cam *Math.cos(mechs.turret_pos());
            telemetry.addData("y_offset: ", -((length_of_beam_score *Math.sin(mechs.turret_pos()))/full_extension_inches) * full_exension_ticks);
            telemetry.addData("turret_pos: ", (Math.acos((x_offset)/ length_of_beam_score)));

            telemetry.update();

            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                mechs.swivelMove(angleFinder.getAngleOfSample()/180 * 0.6 + 0.2);
            }
            if(currentGamepad1.a&&!previousGamepad1.a){
                mechs.extendo_arm_intake();

            }
            mechs.swivelMove(testpos);

        }

    }
}
