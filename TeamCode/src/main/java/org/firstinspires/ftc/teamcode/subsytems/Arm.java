package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    @Config
    public static class ArmParams2024 {
        // Lift constants
        public static double PositionCoefficient = 0.01;
        public static int PositionTolerance = 25;
        public static int LiftStowedTicks = 10; // The lowest position should be greater than 0 to avoid the robot hitting itself
        public static int LowScoreTicks = 500;
        public static int MidScoreTicks = 1000;
        public static int HighScoreTicks = 2900;

        // Shoulder constants
        public static double ShoulderMinAngleDeg= -180;
        public static double ShoulderMaxAngleDeg = 180;
        public static double ShoulderStowed = 0.28;
        public static double ShoulderScore = 0.50;

        // Holder constants
        public static double HolderMinAngleDeg = -180;
        public static double HolderMaxAngleDeg = 180;
        public static double HolderOpen = 0;
        public static double HolderClosed = 0;

    }

    public enum Extension {
        STOW, LOW_SCORE, MID_SCORE, HIGH_SCORE
    }

    public enum ShoulderRotation {
        HORIZONTAL, SCORE
    }

    private MotorEx m_lift;
    private ServoEx m_holder;
    private ServoEx m_shoulder;

    public Arm(HardwareMap hardwareMap) {
        m_lift = new MotorEx(hardwareMap, "lift");
        m_lift.setRunMode(Motor.RunMode.PositionControl);
        m_lift.setInverted(true);

        m_shoulder = new SimpleServo(hardwareMap, "shoulder", ArmParams2024.ShoulderMinAngleDeg, ArmParams2024.ShoulderMaxAngleDeg);
        m_holder = new SimpleServo(hardwareMap, "holder", ArmParams2024.HolderMinAngleDeg, ArmParams2024.HolderMaxAngleDeg);
    }

    public void setLiftHeight(int ticks) {
        m_lift.setTargetPosition(ticks);
    }

    public void moveLift(double power) {
        m_lift.set(power);
    }

    private void updateValues() {
        m_lift.setPositionTolerance(ArmParams2024.PositionTolerance);
        m_lift.setPositionCoefficient(ArmParams2024.PositionCoefficient);
        m_shoulder.setRange(ArmParams2024.ShoulderMinAngleDeg, ArmParams2024.ShoulderMaxAngleDeg);
    }

    public boolean atLiftTarget() {
        return m_lift.atTargetPosition();
    }

    public void setShoulderPosition(double position) {
        m_shoulder.setPosition(position);
    }

    public void setHolderPosition(double position) {
        m_holder.setPosition(position);
    }

    public double getShoulderPosition() {
        return m_shoulder.getPosition();
    }


    @Override
    public void periodic() {
        updateValues();
        TelemetryPacket p = new TelemetryPacket();
        p.put("Holder position", m_holder.getPosition());
        p.put("Lift position", m_lift.getCurrentPosition());
        p.put("Shoulder position", m_shoulder.getPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
