package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class OdoConfigs {
    private double m_leftPositionMeters = 0;
    private double m_rightPositionMeters = 0;
    private double m_perpPositionMeters = 0;
    private double m_deadWheelRadiusMeters = 0;
    private double m_encoderMetersPerTick = 0;
    private double m_ticksPerRevolution = 0;
    private Pose2d m_initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    public OdoConfigs() {

    }

    public OdoConfigs leftPositionMeters(double pos) {
        m_leftPositionMeters = pos;
        return this;
    }

    public OdoConfigs rightPositionMeters(double pos) {
        m_rightPositionMeters = pos;
        return this;
    }

    public OdoConfigs perpPositionMeters(double pos) {
        m_perpPositionMeters = pos;
        return this;
    }

    public OdoConfigs deadWheelRadiusMeters(double radius) {
        m_deadWheelRadiusMeters = radius;
        return this;
    }

    public OdoConfigs encoderMetersPerTick(double mpt) {
        m_encoderMetersPerTick = mpt;
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

    public double getLeftPositionMeters() {
        return m_leftPositionMeters;
    }

    public double getRightPositionMeters() {
        return m_rightPositionMeters;
    }

    public double getPerpPositionMeters() {
        return m_perpPositionMeters;
    }

    public double getDeadWheelRadiusMeters() {
        return m_deadWheelRadiusMeters;
    }

    public double getEncoderMetersPerTick() {
        return m_encoderMetersPerTick;
    }

    public double getTicksPerRevolution() {
        return m_ticksPerRevolution;
    }

    public Pose2d getInitialPose() {
        return m_initialPose;
    }
}
