package org.firstinspires.ftc.teamcode.aRROD.otherRobots;

import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.PIXELS_PER_INCH;
import static org.firstinspires.ftc.teamcode.aRROD.utils.UTILS.extendo_arm_camera_pos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.aRROD.assets.SampleOrientationAnalysisPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

@Config
@TeleOp
public class servoWithCamera extends LinearOpMode {
    public static double pos = 0;
    final static double ticksInDegree = 0.003611111111;
    SampleOrientationAnalysisPipeline angleFinder;
    VisionPortal visionPortal;
    public static double currentPos = 0;
    public static double previousPos = 0;
    public static double turretPos = 0;
    public static double swivelPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx turret = hardwareMap.get(ServoImplEx.class, "servo_eh_0");
        turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        ServoImplEx extendo_arm = hardwareMap.get(ServoImplEx.class, "servo_eh_2");
        extendo_arm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        ServoImplEx swivel = hardwareMap.get(ServoImplEx.class, "servo_eh_1");
        swivel.setPwmRange(new PwmControl.PwmRange(500, 2500));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();
        angleFinder = new SampleOrientationAnalysisPipeline(telemetry);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(angleFinder)
                .enableLiveView(true)
                .build();
        extendo_arm.setPosition(extendo_arm_camera_pos);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){


                swivel.setPosition(angleFinder.getSwivelAngle()/300);



            telemetry.addData("swivel angle: ", angleFinder.getSwivelAngle());
            telemetry.addData("turret angle: ", angleFinder.getTurretAngle());

            telemetry.update();

        }

    }
}
