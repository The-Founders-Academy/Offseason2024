package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends SubsystemBase {
    @Config
    public static class WristParams2024 {
        // All degrees
        public static double MIN_ANGLE = 0;
        public static double MAX_ANGLE = 300;
        public static double STOW_ANGLE = 0;
        public static double SCORE_ANGLE = 300;
    }


    private ServoEx m_wristServo;
    private double m_targetAngle = 0;

    public Wrist(HardwareMap hardwareMap) {
        m_wristServo = new SimpleServo(hardwareMap, "wrist", WristParams2024.MIN_ANGLE, WristParams2024.MAX_ANGLE);
    }

    public void tuneParams() {
        m_wristServo.setRange(WristParams2024.MIN_ANGLE, WristParams2024.MAX_ANGLE);
    }

    public boolean atTarget() {
        return m_wristServo.getAngle() == m_targetAngle;
    }

    public void setTarget(double angleTarget) {
        m_targetAngle = angleTarget;
        m_wristServo.turnToAngle(m_targetAngle);
    }

    @Override
    public void periodic() {
        tuneParams();
    }
}
