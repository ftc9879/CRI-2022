package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.util.concurrent.TimeUnit;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.RotatedRect;
import org.opencv.core.Core;
import org.opencv.core.Size;

@Autonomous

public class RedLeftAuto extends LinearOpMode {
    OpenCvWebcam webcam; 
    OurPipeline pipeline; 
    
   DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor lift;
    DcMotor deliveryMotor;
    DcMotor intakeMotor;
    Servo armServo; 
    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;
    double divide;
    double straightP;

    // Determines time of stack detection
    double visionReadTime;

    // Variables for tracking the angle of the robot
    double angle;
    String angleVal;
    
    // Setup modifying PID values
    PIDFCoefficients pid;
    
        Acceleration gravity;
    BNO055IMU imu;
    Orientation angles;
    
    ElapsedTime timer; 
     // Prepare to use Vuforia and Tensorflow
    private static final String VUFORIA_KEY = " AWFgCSD/////AAABmYkyQ5CPl0oxgJ1ax3vYAbqHDTIleFfJqDw8oca/v28OosWAbIHMkNSwkFuFnM7FPUcXM9sqqdHfFdyMulVLNQyAVUlboelnnXfdw3EkqFCQcF0q6EoJydb2+fJE8fWNLGOrvxZm9rkSX0NT9DVdE6UKfyc/TVpYTYaLegPitiLRpvG4P2cHsHhtUQ48LCuuPN2uFdC1CAJ6YRYtc7UMiTMZw8PyCKM1tlcG6v4dugoERLcoeX2OVA9eFJ2w89/PNK7rzNsLmo4OugTh3bztARq6S7gl+Q/DbscZ3/53Vg+1N4eIXZh/LJwJK6ZJxetftvcXBHi9j9f9T6/ghhY0szUzLmAoKlAO+0XXebOtXKad ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
     public RedLeftAuto() {
        straightP = 0.025;
        visionReadTime = 0.1;
        timer = new ElapsedTime();
    }
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
      "Ball",
      "Cube",
      "Duck",
      "Marker"
    };
    int attempts = 150000;
        int[] votes = {0, 0 ,0};
        String position;
        ExposureControl myExposureControl;
    @Override
    public void runOpMode() throws InterruptedException {


   leftFront = hardwareMap.dcMotor.get("LF");
   rightFront = hardwareMap.dcMotor.get("RF");
   leftBack = hardwareMap.dcMotor.get("LB");
   rightBack = hardwareMap.dcMotor.get("RB");
   lift = hardwareMap.dcMotor.get("L");
   deliveryMotor = hardwareMap.dcMotor.get("DM");
   intakeMotor = hardwareMap.dcMotor.get("IM");
      armServo = hardwareMap.get(Servo.class, "AS");

   leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
   
// Initialize the gyro
        final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Create a timer
        timer = new ElapsedTime();

        // Initialize the vision system
        initializeVision();
        myExposureControl = vuforia.getCamera().getControl(ExposureControl.class); 
        myExposureControl.setMode(ExposureControl.Mode.Manual);
        myExposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        
        pipeline = new OurPipeline();
        webcam.setPipeline(pipeline);
        
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened(){
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            
            @Override
            public void onError(int errorCode){
                System.out.println("Error: " + errorCode);
            }
        });
        armServo.setPosition(.6);
        // Let the drivers know the robot is ready
        telemetry.addData("ROBOT IS READY", "PRESS START TO BEGIN");
        telemetry.update();

        // Start when the play button is pressed
        waitForStart();

        armMove('u','m',1);
        waiting(2);
        determineTSEPosition();
        waiting(.5);
        
        telemetry.addData("position", position);
        telemetry.update();
        if(position == "left"){
            armMove('d','l',.75);
            waiting(.25);
            moveStraight('f',350,0,.5);
            waiting(0.25);
            pointTurn('l',25,.4);
            waiting(0.25);
            moveStraight('f',1300,35,.5);
            waiting(0.25);
            pointTurn('r',10,.4);
            waiting(.25);
            moveStraight('f',1150,0,.5);
            waiting(.25);
            pointTurn('r',-80,.4);
            waiting(.25);

            moveStraight('f',750,-90,.4);
            waiting(1);
            intakeMotor.setPower(-.5);
            waiting(1);
            intakeMotor.setPower(0);
            waiting(.25);
            moveStraight('b',-1600,-90,.5);
            waiting(.25);
            
            pointTurn('l',-10,.4);
            waiting(.25);
            moveStraight('b',-2400,0,.5);
            waiting(.25);
            moveStraightTimer('b',-300,0,.25,1.2);
            deliveryMotor.setPower(-0.5);
            waiting(4);
            waiting(.25);
            moveStraight('f',1500,0,.5);
            waiting(0.25);
            armMove('d','b',.5);
        }
        else if(position == "middle"){  
            moveStraight('f',350,0,.5);
            waiting(0.25);
            pointTurn('l',25,.4);
            waiting(0.25);
            moveStraight('f',1300,35,.5);
            waiting(0.25);
            pointTurn('r',10,.4);
            waiting(.25);
            moveStraight('f',1100,0,.5);
            waiting(.25);
            pointTurn('r',-80,.4);
            waiting(.25);

            moveStraight('f',925,-90,.4);
            waiting(1);
            intakeMotor.setPower(-.5);
            waiting(1);
            intakeMotor.setPower(0);
            waiting(.25);
            moveStraight('b',-1625,-90,.5);
            waiting(.25);
            
            pointTurn('l',-10,.4);
            waiting(.25);
            moveStraight('b',-2500,0,.5);
            waiting(.25);
            moveStraightTimer('b',-300,0,.25,1.2);
            deliveryMotor.setPower(-0.5);
            waiting(4);
            waiting(.25);
            moveStraight('f',1550,0,.5);
            waiting(0.25);
            armMove('d','b',.5);
        }
        else {  
            moveStraight('f',2750,0,.5);
            waiting(0.25);
            pointTurn('r',-80,.40);
            waiting(0.25);
            armMove('u','h',1);
            waiting(.25);
            moveStraight('f',500,-90,.5);
            waiting(.25);
            intakeMotor.setPower(-.5);
            waiting(1);
            intakeMotor.setPower(0);
            
            waiting(.5);
            moveStraight('b',-1950,-90,.5);
            waiting(.25);
            
            pointTurn('l',-10,.4);
            waiting(.25);
            moveStraight('b',-2500,0,.5);
            waiting(.25);
            moveStraightTimer('b',-300,0,.25,1.2);
            deliveryMotor.setPower(-0.5);
            waiting(4);
            waiting(.25);
            moveStraight('f',1400,0,.5);
            waiting(0.25);
            armMove('d','b',.5);
        }
          
        



}




    // A method for initializing Vuforia
    private void initVuforia() {
         
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    // A method for initializing TensorFlow
    private void initTfod() {
 int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.50f;
       tfodParameters.isModelTensorFlow2 = true;
       tfodParameters.inputSize = 320;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    // Methods used for reading gyro input
    String formatAngle(final AngleUnit angleUnit, final double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(final double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    void determineTSEPosition(){
        position = pipeline.positionDetected();
        webcam.stopStreaming();
    }

    // A method for moving the robot straight based on direction, encoders, the angle to hold, and motor power
    void moveStraight(final char fb, final int encoderCount, final double holdAngle, final double motorPower) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25 && opModeIsActive()) {
        }
        if (fb == 'f') {
            while (leftBack.getCurrentPosition() < encoderCount && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                final String angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                double angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif;
                }
                leftFront.setPower(motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(-motorPower + (angle - holdAngle) * straightP);
            }
        } else {
            while (leftBack.getCurrentPosition() > encoderCount && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif2 = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif2;
                }
                leftFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(motorPower + (angle - holdAngle) * straightP);
            }
        }
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }
    
    void moveStraightTimer(final char fb, final int encoderCount, final double holdAngle, final double motorPower, final double time) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25 && opModeIsActive()) {
        }
        if (fb == 'f') {
            while (leftBack.getCurrentPosition() < encoderCount && opModeIsActive() && timer.time()<time) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                final String angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                double angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif;
                }
                leftFront.setPower(motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(-motorPower + (angle - holdAngle) * straightP);
            }
        } else {
            while (leftBack.getCurrentPosition() > encoderCount && opModeIsActive() && timer.time()<time) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                if (holdAngle - angle > 180.0 && holdAngle > 0.0 && angle < 0.0) {
                    final double dif2 = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif2;
                }
                leftFront.setPower(-motorPower + (angle - holdAngle) * straightP);
                leftBack.setPower(-motorPower + (angle - holdAngle) * straightP);
                rightFront.setPower(motorPower + (angle - holdAngle) * straightP);
                rightBack.setPower(motorPower + (angle - holdAngle) * straightP);
            }
        }
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }

    // A method for point turning based on direction, the angle to turn to, and motor power
    void pointTurn(final char lr, final double targetAngle, final double motorPower) {
        if (lr == 'l') {
            while (angle < targetAngle && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                if (targetAngle == 180.0 && angle < 0.0) {
                    final double dif = 180.0 - Math.abs(angle);
                    angle = 180.0 + dif;
                }
                leftBack.setPower(-motorPower);
                leftFront.setPower(-motorPower);
                rightBack.setPower(-motorPower);
                rightFront.setPower(-motorPower);
            }
        } else {
            while (angle > targetAngle && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftBack.setPower(motorPower);
                leftFront.setPower(motorPower);
                rightBack.setPower(motorPower);
                rightFront.setPower(motorPower);
            }
        }
        leftBack.setPower(0.0);
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        rightFront.setPower(0.0);
    }

    // A method for initializing all vision 
    void initializeVision() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
        }
    tfod.setZoom(1, 16.0/9.0);
    }
      // A method that implements a pause
    void waiting(final double waittime) {
        if (waittime == 0) {
            return;
        }
        timer.startTime();
        timer.reset();
        while (timer.time() < waittime && opModeIsActive()) {
        }
    }

    // A method that allows the robot to strafe based on direction, encoder counts, motor power, and an angle to hold
    void strafe(final char lr, final int encoderCounts, final double motorPower, final double holdAngle) {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        timer.startTime();
        timer.reset();
        while (timer.time() < 0.25 && opModeIsActive()) {
        }
        if (lr == 'l') {
            leftFront.setPower(-motorPower);
            leftBack.setPower(motorPower);
            rightFront.setPower(-motorPower);
            rightBack.setPower(motorPower);
            while (leftBack.getCurrentPosition() < encoderCounts && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftFront.setPower(-motorPower + (angle - holdAngle) * 0.004);
                leftBack.setPower(motorPower + (angle - holdAngle) * 0.004);
                rightFront.setPower(-motorPower + (angle - holdAngle) * 0.004);
                rightBack.setPower(motorPower + (angle - holdAngle) * 0.004);
            }
        }
        if (lr == 'r') {
            leftFront.setPower(motorPower);
            leftBack.setPower(-motorPower);
            rightFront.setPower(motorPower);
            rightBack.setPower(-motorPower);
            while (leftBack.getCurrentPosition() > -encoderCounts && opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleVal = formatAngle(angles.angleUnit, angles.firstAngle);
                angle = Float.parseFloat(angleVal);
                leftFront.setPower(motorPower + (angle - holdAngle) * 0.004);
                leftBack.setPower(-motorPower + (angle - holdAngle) * 0.004);
                rightFront.setPower(motorPower + (angle - holdAngle) * 0.004);
                rightBack.setPower(-motorPower + (angle - holdAngle) * 0.004);
            }
        }
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);
    }
    void armMove(final char ud, final char lmh, final double motorPower){
        if(ud == 'u'){
            if(lmh == 'l'){
                while(lift.getCurrentPosition()<500){
                    // used gear 500 
                    lift.setPower(motorPower);
        
                }
            }
            else if(lmh == 'm'){
                while(lift.getCurrentPosition()<1000){
                // used gear 950 
                lift.setPower(motorPower);
                }
            }
            else if(lmh == 'h'){
                while(lift.getCurrentPosition()<1300){
                // used gear 1300
                lift.setPower(motorPower);

                }    
            }
        }
        else {
            if(lmh == 'l'){
                while(lift.getCurrentPosition()>850){
                    // used gear 700 
                    lift.setPower(-motorPower);

                }
            }
            else if(lmh == 'm'){
                while(lift.getCurrentPosition()>900){
                    // used gear 900 
                    lift.setPower(-motorPower);
                } 
            }
            else {
                while(lift.getCurrentPosition()>100){
                    // used gear 100 
                    lift.setPower(-motorPower);
                }
            }

        }
        lift.setPower(0);

    }
    
    class OurPipeline extends OpenCvPipeline
    {
        boolean viewportPaused;
        Mat cbMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();
        
        ArrayList<MatOfPoint> contours = new ArrayList();

        /*
         * Threshold values
         */
        final int CB_CHAN_MASK_THRESHOLD = 80;
        final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
        
        final Scalar TEAL = new Scalar(3, 148, 252);
        final Scalar PURPLE = new Scalar(158, 52, 235);
        final Scalar RED = new Scalar(255, 0, 0);
        final Scalar GREEN = new Scalar(0, 255, 0);
        final Scalar BLUE = new Scalar(0, 0, 255);
        
        
        final int CONTOUR_LINE_THICKNESS = 2;
        final Scalar lower = new Scalar(0.0, 150, 0);
        final Scalar upper = new Scalar(255.0, 255, 140);
        
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
        
        String TSEposition;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {   
            Rect maxRect = new Rect();
            double maxArea = 0;
            
            contours = findContours(input);
            for(MatOfPoint contour : contours){
                Point[] contourArray = contour.toArray();
                if (contourArray.length >= 15){
                    MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                    Rect rect = Imgproc.boundingRect(areaPoints);
                    if(rect.area() > maxArea && rect.y + rect.height > 180){
                        maxArea = rect.area();
                        maxRect = rect; 
                    }
                }
            }
            
            if (maxRect.x < 100 && maxRect.area() > 4500) {
                TSEposition = "left";
            } else if (maxRect.x > 100 && maxRect.area() > 4500) {
                TSEposition = "middle";
            } else {
                TSEposition = "right";
            }
            
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Contours: ", pipeline.getContours());
            telemetry.addData("Rect x: ", maxRect.x);
            telemetry.addData("Rect y: ", maxRect.y + maxRect.height);
            telemetry.addData("Position: ", TSEposition);
            telemetry.addData("Size: ", maxRect.area());
            telemetry.update();
            
            
            
            
            return input; 
            
            
        }
        
        public String positionDetected(){
            return TSEposition; 
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
        
        ArrayList<MatOfPoint> findContours(Mat input){
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            // Convert the input image to YCrCb color space, then extract the Cb channel
            Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
            
            Core.inRange(cbMat, lower, upper, cbMat);

            Imgproc.morphologyEx(cbMat, cbMat, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(cbMat, cbMat, Imgproc.MORPH_CLOSE, new Mat());
            
            Imgproc.GaussianBlur(cbMat, cbMat, new Size(5.0, 15.0), 0.00);
            
            // Ok, now actually look for the contours! We only look for external contours.
            Imgproc.findContours(cbMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            Imgproc.drawContours(input, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }
        
        
        public ArrayList<MatOfPoint> getContours(){
            return contours; 
        }
    }
}
