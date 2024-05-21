package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.List;

public class opencv extends LinearOpMode {
    // X and Y coordinates of Camera || TODO Replace with actual values
    double cX = 0;
    double cY = 0;

    // width of Camera || TODO Replace with actual value
    double width = 0;

    private OpenCvCamera controlHubCam;
    private static final int cameraWidth = 1280;
    private static final int cameraHeight = 720;
    // These values are for the back camera, if we get a front camera remember to check specific values for that


    // NOT CORRECT NUMBERS
    public static final double realUnitObjectWidth = 3.75; // TODO Replace with correct width of object in real units.
    public static final double focalLength = 728; // TODO Replace with correct Focal Length of Camera


    @Override
    public void runOpMode() throws InterruptedException {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // streams it on FTC Dashboard website


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")"); // Prints X and Y coordinate
            telemetry.addData("Distance in Inch", (getDistance(width))); // Prints distance from object in inches
            telemetry.update(); // updates telemetry


        }
        controlHubCam.stopStreaming(); // When opMode is stopped, camera will stop streaming and stop updating telemetry

    }


    private void initOpenCV() {
        // This function is  called in the runOpMode function above.

        // Creates instance of Camera, with name "CameraMonitorViewId"
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "CameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // assigns controlHubCam to a new webcam instance named "Webcam 1", uses cameraMonitorViewId which was created above for this
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        /* Creates new OpenCV image processing pipeline, the class is below as well as instructions for the class.
        When the function is run it starts doing this pipeline as per the instructions below in the class  */

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());
    }


    class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocesses the frame to detect yellow areas
            Mat yellowMask = preprocessFrame(input);

            // Finds contours (areas) of the yellow regions it detected
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


            /* Finds the largest yellow contour and assigns it to "largestContour",
               using the findLargestContour method which is written below */
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);

                // Calculate the width of the bounding box, using the calculateWidth method which is written below
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                // Calculate the centroid (center point) of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;

        }

        // Method for preprocessing Frames, which is called in YellowBlobDetectionPipeline function.
        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);


            // Scalars contain HSV values, values between 100 and 180 contain the values that red is in.
            // TODO may need to change the values of 100-180 to something else, values copied from tutorial
            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;

        }

        // Method for finding the largest area of yellow, used in the function YellowBlobDetectionPipeline function.
        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }


        // Method used to calculate the width of bounding box, (outline being drawn along the largest blob)
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }


    }

    // Used in the initial RunOpMode command to display live telemetry data, as well as displaying it as a label next to the contour box in the YellowBlobDetectionPipeline function.
    private static double getDistance(double width) {

        double distance = (realUnitObjectWidth * focalLength) / width; // THE FORMULA! (Again remember to change realUnitObjectWidth to correct width of team prop)
        return distance;
    }


}
