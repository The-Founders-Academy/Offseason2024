package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class TeamPropProcessor implements VisionProcessor {

    Rect LEFT_RECT;
    Rect MIDDLE_RECT;
    Rect RIGHT_RECT;


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Creating Rectangles to fill camera
        LEFT_RECT = new Rect(
                new Point(0,0),                 // Coordinates from top left to bottom right.
                new Point(width * .33, height)
        );

        MIDDLE_RECT = new Rect(
                new Point(width * .33,0),
                new Point(width * .66, height)
        );

        RIGHT_RECT = new Rect(
                new Point(width * .66, 0),
                new Point(width, height)
        );

        // Calling runOpMode function from FindLargestBlob class
        FindRedProp init = new FindRedProp();

        try {
            init.runOpMode();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }


    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /* Use cX and cY from FindRedProp, (coordinates of centroid) to figure out which third of the screen the prop is in.
        cX and cY should be created when the init function in this class is called */
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
