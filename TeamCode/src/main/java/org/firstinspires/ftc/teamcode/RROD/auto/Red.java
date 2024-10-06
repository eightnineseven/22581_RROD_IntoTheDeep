package org.firstinspires.ftc.teamcode.RROD.auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "ExampleInCode", group = "Autonomous")
public class Red extends LinearOpMode {
    public PoseHolder Pose;
    public double armAngle;
    public static DcMotor armPivot;
    public static PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static double target = 0;

    public static double ticks_in_degree = 537.7/180;

    public static int high_chamber_pos = 0;
    public static int resting_pos = 0;



    public class intakeMechanisms {
        private Servo intakePivot;


        public intakeMechanisms(HardwareMap hardwareMap) {
            intakePivot = hardwareMap.get(Servo.class, "servo_eh_1");
            armPivot = hardwareMap.dcMotor.get("motor_eh_0");

        }

        public class retractAllMechanisms implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivot.setPosition(0);
                armAngle = resting_pos;
                target = armAngle;
                while(armAngle >= armPivot.getCurrentPosition()+10 || armAngle<= armPivot.getCurrentPosition()) {
                    armPID();
                }

                return false;
            }
        }

        public Action retractAllMechanisms() {
            return new retractAllMechanisms();
        }

        public class extendAllMechanisms implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivot.setPosition(0);
                armAngle = high_chamber_pos;
                target = armAngle;
                while(armAngle >= armPivot.getCurrentPosition()+10 || armAngle<= armPivot.getCurrentPosition()) {
                    armPID();
                }
                return false;
            }
        }

        public Action extendAllMechanisms() {
            return new extendAllMechanisms();
        }
    }











    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-Pose.startPos.x, -Pose.startPos.y, Math.toRadians(0)));
        intakeMechanisms intakeMechanisms = new intakeMechanisms(hardwareMap);



        // vision here that outputs position


        Action preloadTraj;
        preloadTraj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.preloadPos.x, -Pose.preloadPos.y))
                .build();

        Action intermediate1;
        intermediate1 = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.beforePlowPos.x, -Pose.beforePlowPos.y))
                .build();

        Action beforePlow1Traj;
        beforePlow1Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.beforeSample1Pos.x, -Pose.beforeSample1Pos.y))
                .build();

        Action plow1Traj;
        plow1Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.plowSample1Pos.x, -Pose.plowSample1Pos.y))
                .build();

        Action beforePlow2Traj;
        beforePlow2Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.beforeSample2Pos.x, -Pose.beforeSample2Pos.y))
                .build();

        Action plow2Traj;
        plow2Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.plowSample2Pos.x, -Pose.plowSample2Pos.y))
                .build();

        Action beforePlow3Traj;
        beforePlow3Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.beforeSample3Pos.x, -Pose.beforeSample3Pos.y))
                .build();

        Action plow3Traj;
        plow3Traj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.plowSample3Pos.x, -Pose.plowSample3Pos.y))
                .build();

        Action parkTraj;
        parkTraj = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-Pose.parkPos.x, -Pose.parkPos.y))
                .build();




        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(intakeMechanisms.retractAllMechanisms());




        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(
                        preloadTraj,
                        intakeMechanisms.retractAllMechanisms(),
                        intermediate1,
                        beforePlow1Traj,
                        plow1Traj,
                        beforePlow2Traj,
                        plow2Traj,
                        beforePlow3Traj,
                        plow3Traj
                )
        );

    }
    public static void armPID(){
            controller.setPID(p, i, d);
            int armPos = armPivot.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double power = pid + ff;
            armPivot.setPower(power);



    }

}