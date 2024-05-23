package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// TODO: Tune lift parameters
public class Lift extends SubsystemBase {
    @Config
    public static class LiftParams2024 {
        public static double PositionCoefficient = 0;
        public static int PositionTolerance = 25;
        public static int StowedTicks = 10; // The lowest position should be greater than 0 to avoid the robot hitting itself
        public static int LowScoreTicks = 5000;
        public static int MidScoreTicks = 10000;
        public static int HighScoreTicks = 12000;
    }

    private MotorEx m_left;
    private MotorEx m_right;

    public Lift(HardwareMap hardwareMap, String leftName, String rightName) {
        m_left = new MotorEx(hardwareMap, leftName);
        m_right = new MotorEx(hardwareMap, rightName);

        // TODO: One of these motors needs to be reversed
        m_left.setRunMode(Motor.RunMode.PositionControl);
        m_right.setRunMode(Motor.RunMode.PositionControl);

        m_left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setExtension(int ticks) {
        m_left.set(ticks);
        m_right.set(ticks);
    }

    public void tunePIDs() {
        m_left.setPositionCoefficient(LiftParams2024.PositionCoefficient);
        m_right.setPositionCoefficient(LiftParams2024.PositionCoefficient);
        m_left.setPositionTolerance(LiftParams2024.PositionTolerance);
        m_right.setPositionTolerance(LiftParams2024.PositionTolerance);
    }

    @Override
    public void periodic() {
        tunePIDs();
    }

}
