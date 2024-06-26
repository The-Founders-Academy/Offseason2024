package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
// Add DriverStation import statement back eventually
public class DetectTeamPropPipeline extends OpenCvPipeline {
    public static final double ProportionLeft = 0.33;
    public static final double ProportionCenter = 0.34;
    public static final double ProportionRight = 0.33;
    private int m_channel;
    private double m_leftMean = 0;
    private double m_centerMean = 0;
    private double m_rightMean = 0;


    public enum PropZone  {
        LEFT, CENTER, RIGHT
    }

    public DetectTeamPropPipeline() {
    }


    @Override
    public void init(Mat mat) {
        int m_channel = 1; // Replace with actual Team value.
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb); // This may be a problematic call (input to input)

        Rect leftRect = new Rect(
                (int) 0, (int) (input.height() * 0.5),
                (int) (input.width() * ProportionLeft), (int) (input.height() * 0.5));
        Rect centerRect = new Rect(
                (int) (input.width() * ProportionLeft), (int) (input.height() * 0.5),
                (int) (input.width() * ProportionCenter), (int) (input.height() * 0.5));
        Rect rightRect = new Rect(
                (int) (input.width() * (ProportionLeft + ProportionCenter)), (int) (input.height() * 0.5),
                (int) (input.width() * ProportionRight), (int) (input.height() * 0.5));

        Mat leftMat = input.submat(leftRect);
        Mat centerMat = input.submat(centerRect);
        Mat rightMat = input.submat(rightRect);

        // Three clones may strain on resources
        Mat extractedLeft = leftMat.clone();
        Mat extractedCenter = centerMat.clone();
        Mat extractedRight = rightMat.clone();

        Core.extractChannel(leftMat, extractedLeft, m_channel);
        Core.extractChannel(centerMat, extractedCenter, m_channel);
        Core.extractChannel(rightMat, extractedRight, m_channel);

        Scalar meanLeft = Core.mean(extractedLeft);
        Scalar meanCenter = Core.mean(extractedCenter);
        Scalar meanRight = Core.mean(extractedRight);

        m_leftMean = meanLeft.val[0];
        m_centerMean = meanCenter.val[0];
        m_rightMean = meanRight.val[0];

        // Call getPropZone() and use its result
        PropZone zone = getPropZone();

        return input; // Return the modified input (or null if not modified)
    }

    public PropZone getPropZone() {
        PropZone zone = PropZone.LEFT;

        if(m_leftMean < m_centerMean) zone = PropZone.CENTER;
        if(m_leftMean < m_centerMean && m_centerMean < m_rightMean) zone = PropZone.RIGHT;

        return zone;
    }
}
