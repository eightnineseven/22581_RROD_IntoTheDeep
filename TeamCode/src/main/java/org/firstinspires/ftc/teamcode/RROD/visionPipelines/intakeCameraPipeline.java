package org.firstinspires.ftc.teamcode.opmodes.cameraPipelines;







import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public abstract class intakeCameraPipeline implements VisionProcessor {
    //THIS IS JUST THE PIPELINE NOT ANY RUNABLE CODE. The camera runs this code and outputs a value that we'll determine later
    //while yes its a camera video, really a video is just a bunch of pictures so this code analyzes picture by picture and thus the whole camera stream
    private static final boolean DEBUG = true;
    //The positions on the camera for which we begin to draw the boxes we'll use to look at the color of an object
    public static int redLeftX = (int) (815);
    public static int redLeftY = (int) (550);
    public static int redCenterX = (int) (1365);
    public static int redCenterY = (int) (475);
    public static int blueLeftX = (int) (240);
    public static int blueLeftY = (int) (525);
    public static int blueCenterX = (int) (925);
    public static int blueCenterY = (int) (475);
    public static int leftWidth = (int) (175);
    public static int leftHeight = (int) (100);
    public static int centerWidth = (int) (125);
    public static int centerHeight = (int) (125);
    //How much of a color is actually in the box we drew on the camera frame
    public static double BLUE_TRESHOLD = 70;
    public static double RED_TRESHOLD = 100;
    private final Mat hsv = new Mat();
    //what is the color
    public double leftColor = 0.0;
    public double centerColor = 0.0;
    //color represented as R, G, B (could be like B, G, R or some BS idk ill have to look it up)
    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);
    public Scalar right = new Scalar(0,0,0);
    Telemetry telemetry;


    //initializing the pipeline which is THIS WHOLE CODE TAB
    public intakeCameraPipeline() {
        this(null);
    }

    public intakeCameraPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    //initiation of the camera that will be used in the actual code we run
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }
    //here's the pipeline we have to code
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        //rect is literally a rectangle drew onto the camera screen
        Rect leftZoneArea;
        Rect centerZoneArea;
        Rect rightZoneArea;

// determining the position of the rectangles
        leftZoneArea = new Rect(2,2,2,2);
        centerZoneArea = new Rect(1,1,1,1);
        rightZoneArea = new Rect(0,0,0,0);


// This is creating finalizing the rectangle as a subset of the original picture
        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);
        Mat rightZone = frame.submat(rightZoneArea);



// the color the rectangle we drew shows up as on the camera
        Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
        Imgproc.rectangle(frame, rightZoneArea, new Scalar(255,255,255), 2);


// takes the average color value of each section
        left = Core.mean(leftZone);
        center = Core.mean(centerZone);
        right = Core.mean(rightZone);

        if (telemetry != null) {
            //outputs the color of each section to the driver station
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("rightColor", right.toString());

            telemetry.update();
        }

// Determine and set all conditions for what happens depending on the 3 zone colors


//ends the pipeline for this frame
        leftZone.release();
        centerZone.release();
        rightZone.release();

        return null;
    }


    //Can be called in teleop to tell drivers if there is a piece to intake
    public boolean isThereAPiece() {
        return true;
    }
    //Also to be called in teleop to tell drivers what color the piece will be
    public int colorOfPiece(){
        return 1 ;
    }

}