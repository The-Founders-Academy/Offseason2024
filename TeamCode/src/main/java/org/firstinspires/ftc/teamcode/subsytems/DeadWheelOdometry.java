package org.firstinspires.ftc.teamcode.subsytems;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;

import org.firstinspires.ftc.teamcode.util.DriverStation;

public class DeadWheelOdometry {
    private Encoder m_left;
    private Encoder m_right;
    private Encoder m_perp;
    private double m_previousLeft = 0;
    private double m_previousRight = 0;
    private double m_previousPerp = 0;
    private double L; // Track width
    private double B; // Perp encoder position
    private double R; // Wheel radius
    private double N; // Ticks per rev
    private double cm_per_tick;
    private double theta = 0;

    public DeadWheelOdometry(Motor left, Motor right, Motor perp, OdoConfigs configs) {
        m_left = left.encoder;
        m_right = right.encoder;
        m_perp = perp.encoder;

        L = configs.getTrackWidthCentimeters();
        B = configs.getPerpOffset();
        R = configs.getDeadWheelRadiusCentimeters();
        N = configs.getTicksPerRevolution();
        cm_per_tick = 2 * Math.PI * R / N;
    }

    /**
     * Evolves the robot pose with time
     * @return
     */
    public Pose2d update() {
        double dL = (double) m_left.getPosition() - m_previousLeft;
        double dR = (double) m_right.getPosition() - m_previousRight;
        double dP = (double) m_perp.getPosition() - m_previousPerp;

        double dTheta = cm_per_tick * (dR - dL) / L;
        theta += dTheta;
        DriverStation.getInstance().getTelemetry().clearAll();
        DriverStation.getInstance().getTelemetry().addData("Theta", Math.toDegrees(theta));
        DriverStation.getInstance().getTelemetry().update();
        return null;
    }

}
