package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.DriveVectorScaler;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class SwerveFollower extends com.pedropathing.follower.Follower {
    private DriveVectorScaler driveVectorScaler;
    private List<DcMotorEx> motors;
    private List<Servo> servos;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private Servo leftFrontServo;
    private Servo leftRearServo;
    private Servo rightFrontServo;
    private Servo rightRearServo;
    private double[] rollerAngles = {45,135,135,45};

    public SwerveFollower(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void initialize() {
        super.initialize();
        //servo locals
    }

    @Override
    public void update() {
        super.update();
        this.motors = Arrays.asList(this.leftFront, this.leftRear, this.rightFront, this.rightRear);
        this.servos = Arrays.asList(this.leftFrontServo, this.leftRearServo, this.rightFrontServo, this.rightRearServo);
       double[] wheelPower  = this.driveVectorScaler.getDrivePowers(this.getCorrectiveVector(), this.getHeadingVector(), this.getDriveVector(), this.poseUpdater.getPose().getHeading());
       for(int i = 0; i <= 2; i +=2){
           double servoPos = (((wheelPower[i] * rollerAngles[i]) + (wheelPower[i++] * rollerAngles[i++])) + 180) / 360;
           ((Servo)this.servos.get(i)).setPosition(servoPos);
           ((Servo)this.servos.get(i++)).setPosition(servoPos);
           ((DcMotorEx)this.motors.get(i)).setPower(FollowerConstants.maxPower);
           ((DcMotorEx)this.motors.get(i++)).setPower(FollowerConstants.maxPower);


       }
    }
    public void setServoAngle(double[] wheelPowers){

        double[] wheelPower  = wheelPowers;
        for(int i = 0; i <= 2; i +=2){
            double servoPos = (((wheelPower[i] * rollerAngles[i]) + (wheelPower[i++] * rollerAngles[i++])) + 180) / 360;
            ((Servo)this.servos.get(i)).setPosition(servoPos);
            ((Servo)this.servos.get(i++)).setPosition(servoPos);


        }
    }
    public void setServoAngle(double angle){
        for(int i = 0; i < 4; i++){
            ((Servo)this.servos.get(i)).setPosition(angle);
        }
    }
    public void setMotorPower(double power){
        for(int i = 0; i < 4; i++){
            ((DcMotorEx)this.motors).setPower(power);
        }
    }
}
