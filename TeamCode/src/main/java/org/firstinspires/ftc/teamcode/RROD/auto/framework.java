package org.firstinspires.ftc.teamcode.RROD.auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "ExampleInCode", group = "Autonomous")
public class framework extends LinearOpMode {
    public class Lift {
        private final DcMotorEx liftL;
        private final DcMotorEx liftR;
        public HardwareMap hardwareMap;

        public Lift(HardwareMap hardwareMap) {
            liftL = hardwareMap.get(DcMotorEx.class, "motor_eh_1");
            liftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftL.setDirection(DcMotorSimple.Direction.FORWARD);

            liftR = hardwareMap.get(DcMotorEx.class,"motor_ch_1");
            liftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftR.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(0.8);
                    liftR.setPower(0.8);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(-0.8);
                    liftL.setPower(-0.8);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }



        public class placeSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftL.setPower(-0.2);
                    liftR.setPower(-0.2);
                    initialized = true;
                }

                double pos = liftL.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 2900.0) {
                    return true;
                } else {
                    liftL.setPower(0);
                    liftR.setPower(0);
                    return false;
                }
            }
        }
        public Action placeSpecimen() {
            return new placeSpecimen();
        }
    }

    public class intakeMechanisms {
        private Servo linkageL;
        private Servo linkageR;
        private Servo intakePivotL;
        private Servo intakePivotR;
        private CRServo intakePiece;

        public intakeMechanisms(HardwareMap hardwareMap) {
            linkageL = hardwareMap.get(Servo.class, "servo_eh_1");
            linkageR = hardwareMap.get(Servo.class, "servo_ch_1");

            intakePivotL = hardwareMap.get(Servo.class, "servo_eh_1");
            intakePivotR = hardwareMap.get(Servo.class, "servo_ch_1");

            intakePiece = hardwareMap.get(CRServo.class, "servo_ch_1");

        }

        public class retractIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkageR.setPosition(1-0.55);
                linkageL.setPosition(0.55);
                return false;
            }
        }
        public Action retractIntake() {
            return new retractIntake();
        }

        public class extendIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                linkageR.setPosition(1-1.0);
                linkageL.setPosition(1.0);
                return false;
            }
        }
        public Action extendIntake() {
            return new extendIntake();
        }

        public class flipIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivotR.setPosition(1-0.55);
                intakePivotL.setPosition(0.55);
                return false;
            }
        }
        public Action flipIn() {
            return new flipIn();
        }

        public class flipOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePivotL.setPosition(1-1.0);
                intakePivotR.setPosition(1.0);
                return false;
            }
        }
        public Action flipOut() {
            return new flipOut();
        }
        public class pickupPiece implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePiece.setPower(0.6);
                return false;
            }
        }
        public Action pickupPiece() {
            return new pickupPiece();
        }


        public class putInOuttakeBucket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakePiece.setPower(-0.4);
                return false;
            }
        }
        public Action putInOuttakeBucket() {
            return new putInOuttakeBucket();
        }
    }

    public class outtakeMechanisms {
        private Servo outtakePivotL;
        private Servo outtakePivotR;
        private Servo bucketClamp;
        private Servo specimenClamp;

        public outtakeMechanisms(HardwareMap hardwareMap) {
            outtakePivotL = hardwareMap.get(Servo.class, "servo_eh_1");
            outtakePivotR = hardwareMap.get(Servo.class, "servo_ch_1");
            bucketClamp = hardwareMap.get(Servo.class, "servo_eh_1");
            specimenClamp = hardwareMap.get(Servo.class, "servo_eh_1");

        }
        public class flipIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotR.setPosition(1-0.55);
                outtakePivotL.setPosition(0.55);
                return false;
            }
        }
        public Action flipIn() {
            return new flipIn();
        }

        public class flipOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakePivotL.setPosition(1-1.0);
                outtakePivotR.setPosition(1.0);
                return false;
            }
        }
        public Action flipOut() {
            return new flipOut();
        }
        public class openSpecimenClamp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimenClamp.setPosition(1-1.0);
                return false;
            }
        }
        public Action openSpecimenClamp() {
            return new openSpecimenClamp();
        }

        public class closeSpecimenClamp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                specimenClamp.setPosition(1-1.0);
                return false;
            }
        }
        public Action closeSpecimenClamp() {
            return new closeSpecimenClamp();
        }

    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(90)));
        intakeMechanisms intakeMechanisms = new intakeMechanisms(hardwareMap);
        outtakeMechanisms outtakeMechanisms = new outtakeMechanisms(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs position


        Action trajectoryActionPreload;


        trajectoryActionPreload = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(20, 10))
                .build();


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(intakeMechanisms.flipIn());
        Actions.runBlocking(intakeMechanisms.retractIntake());
        Actions.runBlocking(outtakeMechanisms.closeSpecimenClamp());
        Actions.runBlocking(outtakeMechanisms.flipOut());



        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;



        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionPreload,
                        lift.placeSpecimen(),
                        outtakeMechanisms.openSpecimenClamp(),
                        outtakeMechanisms.flipIn(),
                        lift.liftDown(),
                        outtakeMechanisms.flipOut()
                )
        );
    }
}