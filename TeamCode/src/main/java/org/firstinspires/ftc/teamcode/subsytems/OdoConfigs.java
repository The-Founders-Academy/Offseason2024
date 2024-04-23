package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class OdoConfigs {
    private double m_leftPositionCentimeters = 0;
    private double m_rightPositionCentimeters = 0;
    private double m_perpPositionCentimeters = 0;
    private double m_deadWheelRadiusCentimeters = 0;
    private double m_encoderCentimetersPerTick = 0;
    private double m_ticksPerRevolution = 0;
    private double m_trackWidthCentimeters = 0;
    private Pose2d m_initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public OdoConfigs() {

    }

    public OdoConfigs leftPositionCentimeters(double pos) {
        m_leftPositionCentimeters = pos;
        return this;
    }

    public OdoConfigs rightPositionCentimeters(double pos) {
        m_rightPositionCentimeters = pos;
        return this;
    }

    public OdoConfigs perpPositionCentimeters(double pos) {
        m_perpPositionCentimeters = pos;
        return this;
    }

    public OdoConfigs deadWheelRadiusCentimeters(double radius) {
        m_deadWheelRadiusCentimeters = radius;
        return this;
    }

    public OdoConfigs encoderCentimetersPerTick(double mpt) {
        m_encoderCentimetersPerTick = mpt;
        return this;
    }

    public OdoConfigs ticksPerRevolution(double tpr) {
        m_ticksPerRevolution = tpr;
        return this;
    }

    public OdoConfigs initialPose(Pose2d initialPose) {
        m_initialPose = initialPose;
        return this;
    }

    public OdoConfigs trackWidthCentimeters(double width) {
        m_trackWidthCentimeters = width;
        return this;
    }

    public double getLeftPositionCentimeters() {
        return m_leftPositionCentimeters;
    }

    public double getRightPositionCentimeters() {
        return m_rightPositionCentimeters;
    }

    public double getPerpPositionCentimeters() {
        return m_perpPositionCentimeters;
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

    public Pose2d getInitialPose() {
        return m_initialPose;
    }

    public double getTrackWidthCentimeters() {
        return m_trackWidthCentimeters;
    }
}
