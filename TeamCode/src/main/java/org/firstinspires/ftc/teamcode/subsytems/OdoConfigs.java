package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class OdoConfigs {
    private double m_deadWheelRadiusCentimeters = 0;
    private double m_encoderCentimetersPerTick = 0;
    private double m_ticksPerRevolution = 0;
    private double m_trackWidthCentimeters = 0;
    private double m_perpOffset = 0;

    public OdoConfigs() {

    }

    public OdoConfigs deadWheelRadiusCentimeters(double radius) {
        m_deadWheelRadiusCentimeters = radius;
        return this;
    }

    public OdoConfigs ticksPerRevolution(double tpr) {
        m_ticksPerRevolution = tpr;
        return this;
    }

    public OdoConfigs trackWidthCentimeters(double width) {
        m_trackWidthCentimeters = width;
        return this;
    }

    public OdoConfigs perpOffset(double offset) {
        m_perpOffset = offset;
        return this;
    }

    public double getDeadWheelRadiusCentimeters() {
        return m_deadWheelRadiusCentimeters;
    }

    public double getEncoderCentimetersPerTick() {
        return m_encoderCentimetersPerTick;
    }

    public double getTicksPerRevolution() {
        return m_ticksPerRevolution;
    }

    public double getTrackWidthCentimeters() {
        return m_trackWidthCentimeters;
    }

    public double getPerpOffset() {
        return m_perpOffset;
    }
}
