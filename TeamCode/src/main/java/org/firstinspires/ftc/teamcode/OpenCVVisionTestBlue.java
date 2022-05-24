package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import java.util.ArrayList;
import org.opencv.core.RotatedRect;
import org.opencv.core.Core;
import org.opencv.core.Size;

@Autonomous

public class OpenCVVisionTestBlue extends LinearOpMode {
    OpenCvWebcam webcam; 
    OurPipeline pipeline; 
    
    
    @Override
    public void runOpMode(){ 
    
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
        
        telemetry.addLine("Waiting for start >");
        telemetry.update();
        
        
        while(true){
            
            if(gamepad1.a){
                webcam.stopStreaming();
                break;
            }
            
            sleep(100);
        }
        
        waitForStart(); 
        
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
        
        String position;

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
            
            if (maxRect.x > 90 && maxRect.area() > 4000) {
                position = "right";
            } else if (maxRect.x > 0 && maxRect.area() > 4000) {
                position = "middle";
            } else {
                position = "left";
            }
            
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Contours: ", pipeline.getContours());
            telemetry.addData("Rect: ", maxRect.x);
            telemetry.addData("Position: ", position);
            telemetry.addData("Size: ", maxRect.area());
            telemetry.update();
            
            
            
            
            return input; 
            
            
        }
        
        public String positionDetected(){
            return position; 
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
