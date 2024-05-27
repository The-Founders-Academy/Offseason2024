package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: Tune lift parameters
public class Lift extends SubsystemBase {
    @Config
    public static class LiftParams2024 {
        public static double PositionCoefficient = 0.03;
        public static int PositionTolerance = 25;
        public static int StowedTicks = 150; // The lowest position should be greater than 0 to avoid the robot hitting itself
        public static int LowScoreTicks = 5000;
        public static int MidScoreTicks = 10000;
        public static int HighScoreTicks = 12000;
    }

    public enum Extension {
        STOW, LOW_SCORE, MID_SCORE, HIGH_SCORE
    }

    private MotorEx m_left;
    private MotorEx m_right;

    public Lift(HardwareMap hardwareMap, String leftName, String rightName) {
        m_left = new MotorEx(hardwareMap, leftName);
        m_right = new MotorEx(hardwareMap, rightName);
        m_right.setInverted(true);

        m_left.setRunMode(Motor.RunMode.PositionControl);
        m_right.setRunMode(Motor.RunMode.PositionControl);

        m_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetExtension(int ticks) {
        m_left.setTargetPosition(ticks);
        m_right.setTargetPosition(ticks);
    }

    public void setTargetExtension(Extension extension) {
        switch(extension) {
            case STOW:
                setTargetExtension(LiftParams2024.StowedTicks);
                break;
            case LOW_SCORE:
                setTargetExtension(LiftParams2024.LowScoreTicks);
                break;
            case MID_SCORE:
                setTargetExtension(LiftParams2024.MidScoreTicks);
                break;
            case HIGH_SCORE:
                setTargetExtension(LiftParams2024.HighScoreTicks);
                break;
        }
    }

    public void set(double value) {
        m_left.set(value);
        m_right.set(value);
    }

    public void tunePIDs() {
        m_left.setPositionCoefficient(LiftParams2024.PositionCoefficient);
        m_right.setPositionCoefficient(LiftParams2024.PositionCoefficient);
        m_left.setPositionTolerance(LiftParams2024.PositionTolerance);
        m_right.setPositionTolerance(LiftParams2024.PositionTolerance);
    }

    public boolean atSetPoint() {
        return m_left.atTargetPosition() && m_right.atTargetPosition();
    }

    @Override
    public void periodic() {
        tunePIDs();
        TelemetryPacket p = new TelemetryPacket();
        p.put("left position", m_left.encoder.getPosition());
        p.put("right position", m_right.encoder.getPosition());
        p.put("at target", m_left.atTargetPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}