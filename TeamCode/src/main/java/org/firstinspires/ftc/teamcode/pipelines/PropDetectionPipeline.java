package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropDetectionPipeline extends OpenCvPipeline {
    public double cX = 0;
    double cY = 0;

    // Width of Camera || TODO Replace with actual value
    double width = 0;

    private Vision.PropZone m_propZone = null;
    // These values are for the back camera, if we get a front camera remember to check specific values for that
    private static final int cameraWidth = 1280; // Values for EOCV-Sim, actual value is 1280, EOCV-Sim is 640
    private static final int cameraHeight = 720; // Values for EOCV-Sim, actual value is 720, EOCV-Sim is 480

    // NOT CORRECT NUMBERS
    public static final double realUnitObjectWidth = 3.75; // TODO Replace with correct width of object in real units.
    public static final double focalLength = 728; // TODO Replace with correct Focal Length of Camera
    private double m_lowerHue;
    private double m_higherHue;

    public PropDetectionPipeline(double lowerHue, double higherHue) {
        super();
        m_lowerHue = lowerHue;
        m_higherHue = higherHue;
    }
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
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 2);

            // Calculate the width of the bounding box, using the calculateWidth method which is written below
            width = calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            // Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

            // Calculate the centroid (center point) of the largest contour
            Point centroid = calculateCentroid(largestContour);
            cX = centroid.x;
            cY = centroid.y;

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            // Say what third it's in
            LCRvalue(cX);
        }

        return input;
    }

    // Method for preprocessing Frames, which is called in YellowBlobDetectionPipeline function.
    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Scalars contain HSV values, values between 100 and 180 contain the values that red is in.
        // TODO may need to change the values of 100-180 to something else, values copied from tutorial
        Scalar lowerYellow = new Scalar(m_lowerHue, 100, 100);
        Scalar upperYellow = new Scalar(m_higherHue, 255, 255);

        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(5, 5));
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


    // Method to calculate the centroid of a contour
    private Point calculateCentroid(MatOfPoint contour) {
        Point centroid = new Point(0, 0);
        int numPoints = contour.rows();

        for (int i = 0; i < numPoints; i++) {
            double[] point = contour.get(i, 0);
            centroid.x += point[0];
            centroid.y += point[1];
        }

        centroid.x /= numPoints;
        centroid.y /= numPoints;

        return centroid;
    }
    private void LCRvalue(double cord) {
        double coordinate = cord;
        double cordProp = coordinate / cameraWidth;


        if (cordProp <= 0.33) {
            m_propZone = Vision.PropZone.LEFT;
        }
        else if (cordProp > 0.33 && cordProp <= 0.5) {

            m_propZone = Vision.PropZone.CENTER;
        }
        else if (cordProp > 0.5 && cordProp <= 1){

            m_propZone = Vision.PropZone.RIGHT;

        }
    }

    public Vision.PropZone getPropZone() {
        return m_propZone;
    }

    // Used in the initial RunOpMode command to display live telemetry data, as well as displaying it as a label next to the contour box in the YellowBlobDetectionPipeline function.
    private static double getDistance(double width) {
        double distance = (realUnitObjectWidth * focalLength) / width; // THE FORMULA! (Again remember to change realUnitObjectWidth to correct width of team prop)
        return distance;
    }
}
