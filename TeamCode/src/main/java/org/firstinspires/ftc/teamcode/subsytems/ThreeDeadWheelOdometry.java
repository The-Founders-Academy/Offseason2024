package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;

import org.firstinspires.ftc.teamcode.util.DriverStation;

public class ThreeDeadWheelOdometry {
    private Encoder m_left;
    private Encoder m_right;
    private Encoder m_perp;
    private OdoConfigs m_configs;
    private double m_previousTicksLeft = 0;
    private double m_previousTicksRight = 0;
    private double m_previousTicksPerp = 0;
    private double B = 0;
    private double L = 0;
    private double R = 0;
    private double N = 0;
    public double cm_per_tick = 0;
    double testTheta = 0;

    /**
     * @param left The motor whose encoder slot is occupied by the left deadwheel encoder
     * @param right The motor whose encoder slot is occupied by the right deadwheel encoder
     * @param perpendicular The motor whose encoder slot is occupied by the perpendicular encoder
     * @param configs A configs object specifying deadwheel characteristics and initial conditions
     */
    public ThreeDeadWheelOdometry(Motor left, Motor right, Motor perpendicular, OdoConfigs configs) {
        m_left = left.encoder;
        m_left.setDirection(Motor.Direction.REVERSE);
        m_right = right.encoder;
        m_perp = perpendicular.encoder;
        m_configs = configs;

        R = m_configs.getDeadWheelRadiusCentimeters();
        B = m_configs.getPerpPositionCentimeters();
        N = m_configs.getTicksPerRevolution();
        L = m_configs.getTrackWidthCentimeters();
        cm_per_tick = 2 * Math.PI * R / N;
    }

    public Pose2d getVelocities() {
        return null; // TO-DO: Implement
    }

    /**
     * Evolves the robot's field-coordinate pose based on deadwheel speeds
     * @return The new field-coordinate robot pose
     */
    public Pose2d update(Pose2d previousPose) {
        // Calculate how many ticks each deadwheel has traveled since the last time this method was called
        double currentTicksLeft = getEncoderPositionDouble(m_left);
        double currentTicksRight = getEncoderPositionDouble(m_right);
        double currentTicksPerp = getEncoderPositionDouble(m_perp);

        double nL = currentTicksLeft - m_previousTicksLeft;
        double nR = currentTicksRight - m_previousTicksRight;
        double nP = currentTicksPerp - m_previousTicksPerp;

        // Calculate the difference in x, y, and theta since this method was last called
        double dx = cm_per_tick * ((nL + nR) / 2.0);
        double dy = cm_per_tick * (nP - (m_configs.getPerpPositionCentimeters() * ((nR - nL) / L)));
        double dTheta = cm_per_tick * ((nR - nL) / 2.0);

        double x0 = previousPose.getX();
        double y0 = previousPose.getY();
        double theta0 = previousPose.getHeading(); // Radians

        double theta = theta0 + dTheta / 2;
        double nX = x0 + dx * Math.cos(theta) - dy * Math.sin(theta);
        double nY = y0 + dx * Math.sin(theta) + dy * Math.cos(theta);
        double nTheta = theta0 + dTheta;
        testTheta += dTheta * 7.2; // There is a missing factor of 7.2 that is being compensated for here. Where is it supposed to go???

        // Memoize the current deadwheel positions in ticks
        m_previousTicksLeft = currentTicksLeft;
        m_previousTicksRight = currentTicksRight;
        m_previousTicksPerp = currentTicksPerp;

        DriverStation.getInstance().getTelemetry().clearAll();
        DriverStation.getInstance().getTelemetry().addData("diff ticks", nR - nL);
        DriverStation.getInstance().getTelemetry().addData("left ticks", m_left.getPosition());
        DriverStation.getInstance().getTelemetry().addData("right ticks",m_right.getPosition());
        DriverStation.getInstance().getTelemetry().addData("perp ticks", m_perp.getPosition());
        DriverStation.getInstance().getTelemetry().addData("Differential Theta", dTheta);
        DriverStation.getInstance().getTelemetry().addData("nx", nX);
        DriverStation.getInstance().getTelemetry().addData("ny", nY);
        DriverStation.getInstance().getTelemetry().addData("ntheta", nTheta);
        DriverStation.getInstance().getTelemetry().addData("test theta", testTheta);
        DriverStation.getInstance().getTelemetry().update();

        return new Pose2d(nX, nY, Rotation2d.fromDegrees(Math.toDegrees(nTheta)));
    }

    /**
     * Encoders ticks are in int and it is not wise to cast them to double inline, so this function will do it for us
     * @param encoder The encoder to get the position value from
     * @return The encoders current position as a double
     */
    public static double getEncoderPositionDouble(Encoder encoder) {
        return (double) encoder.getPosition();
    }
}
