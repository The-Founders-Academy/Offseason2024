package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;

public class ThreeDeadWheelOdometry {
    private Encoder m_left;
    private Encoder m_right;
    private Encoder m_perp;
    private OdoConfigs m_configs;
    private double m_previousTicksLeft = 0;
    private double m_previousTicksRight = 0;
    private double m_previousTicksPerp = 0;
    private Pose2d m_currentPose;
    private double odoConstant = 0;

    /**
     * @param left The motor whose encoder slot is occupied by the left deadwheel encoder
     * @param right The motor whose encoder slot is occupied by the right deadwheel encoder
     * @param perpendicular The motor whose encoder slot is occupied by the perpendicular encoder
     * @param configs A configs object specifying deadwheel characteristics and initial conditions
     */
    public ThreeDeadWheelOdometry(Motor left, Motor right, Motor perpendicular, OdoConfigs configs) {
        m_left = left.encoder;
        m_right = right.encoder;
        m_right.setDirection(Motor.Direction.REVERSE);
        m_perp = perpendicular.encoder;
        m_configs = configs;

        m_currentPose = configs.getInitialPose();
        odoConstant = 2 * Math.PI * configs.getDeadWheelRadiusMeters() / m_configs.getTicksPerRevolution();
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
        double distBetweenParEnc = Math.abs(m_configs.getLeftPositionMeters() - m_configs.getRightPositionMeters());
        double currentTicksLeft = getEncoderPositionDouble(m_left);
        double currentTicksRight = getEncoderPositionDouble(m_right);
        double currentTicksPerp = getEncoderPositionDouble(m_perp);

        double nL = currentTicksLeft - m_previousTicksLeft;
        double nR = currentTicksRight - m_previousTicksRight;
        double nP = currentTicksPerp - m_previousTicksPerp;

        // Calculate the difference in x, y, and theta since this method was last called
        double dx = odoConstant * ((nL + nR) / 2);
        double dy = odoConstant * (nP - m_configs.getPerpPositionMeters() * ((nR - nL) / distBetweenParEnc));
        double dTheta = odoConstant * ((nR - nL) / 2);

        double x0 = previousPose.getX();
        double y0 = previousPose.getY();
        double theta0 = previousPose.getHeading(); // Radians

        double nX = x0 + dx * Math.cos(theta0) - dy * Math.sin(theta0);
        double nY = y0 + dx * Math.sin(theta0) + dy * Math.cos(theta0);
        double nTheta = theta0 + dTheta;

        // Memoize the current deadwheel positions in ticks
        m_previousTicksLeft = currentTicksLeft;
        m_previousTicksRight = currentTicksRight;
        m_previousTicksPerp = currentTicksPerp;

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
