package org.firstinspires.ftc.teamcode.aRROD.otherRobots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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


    @Override
    public void runOpMode() throws InterruptedException {
        angleFinder = new SampleOrientationAnalysisPipeline();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(angleFinder)
                .enableLiveView(true)
                .build();
        Servo servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
        servo.setPosition((0.73 + 0.08) / 2);
        waitForStart();
        while(opModeIsActive()&&!isStopRequested()){
            if(gamepad1.dpad_up) {
                servo.setPosition(angleFinder.getSwivelAngle() * ticksInDegree+0.08);
            }
            if(gamepad1.dpad_left){
                //smth
            }

            telemetry.addData("angle finder angle: ", angleFinder.getSwivelAngle());
            telemetry.update();

        }

    }
}
