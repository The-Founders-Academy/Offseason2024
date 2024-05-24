package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;


public class Mecanum2024 extends BaseMecanumDrive {

    @Config
    public static class Mecanum2024Params {
        public static double TranslationP = 0.03;
        public static double TranslationI = 0;
        public static double TranslationD = 0;
        public static double RotationP = 0;
        public static double RotationI = 0;
        public static double RotationD = 0;

        public static double TranslationToleranceCentimeters = 0.5;
        public static double RotationToleranceRad = Math.toRadians(3); // 3 Deg
    }

    private Pose2d m_robotPose;
    private Pose2d m_targetPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private HolonomicOdometry m_odo;
    private Encoder left;
    private Encoder right;
    private Encoder horizontal;
    private IMU m_gyro;
    private PIDController m_translationXController;
    private PIDController m_translationYController;
    private PIDController m_rotationController;
    private double m_initialAngleRad;

    public Mecanum2024(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        super(hardwareMap, mecanumConfigs, initialPose);
        m_robotPose = initialPose;


        OdoConfigs odoConfigs = new OdoConfigs()
                .deadWheelRadiusCentimeters(2.4)
                .ticksPerRevolution(2000.0)
                .trackWidthCentimeters(36.83)
                .perpOffset(-20.32);

        double cm_per_tick = 2 * Math.PI * odoConfigs.getDeadWheelRadiusCentimeters() / odoConfigs.getTicksPerRevolution();
        left = m_backRight.encoder.setDistancePerPulse(cm_per_tick);
        left.setDirection(Motor.Direction.REVERSE);
        right = m_frontRight.encoder.setDistancePerPulse(cm_per_tick);
        horizontal = m_frontLeft.encoder.setDistancePerPulse(cm_per_tick);

        m_odo = new HolonomicOdometry(
                left::getDistance,
                right::getDistance,
                horizontal::getDistance,
                odoConfigs.getTrackWidthCentimeters(),
                odoConfigs.getPerpOffset()
        );

        m_gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        m_gyro.initialize(myIMUparameters);

        // m_odo is tracking heading / angle offset, so set its initial rotation to 0
        m_odo.updatePose(new Pose2d(initialPose.getX(), initialPose.getY(), Rotation2d.fromDegrees(0)));

        m_robotPose = initialPose;
        m_initialAngleRad = initialPose.getHeading();

        // These zeroes are replaced with real values as soon as tunePIDs() gets called
        m_translationXController = new PIDController(0, 0, 0);
        m_translationYController = new PIDController(0, 0, 0);
        m_rotationController = new PIDController(0, 0, 0);
    }

    @Override
    public Rotation2d getHeading() {
        return m_odo.getPose().getRotation();
    }

    public void setTargetPose(Pose2d targetPose) {
        m_targetPose = targetPose;
        m_translationXController.setSetPoint(m_targetPose.getX());
        m_translationYController.setSetPoint(m_targetPose.getY());
        m_rotationController.setSetPoint(m_targetPose.getHeading());
    }

    public boolean atTargetPose() {
        return (m_translationXController.atSetPoint() && m_translationYController.atSetPoint() && m_rotationController.atSetPoint());
    }

    public void resetPIDs() {
        m_translationXController.reset();
        m_translationYController.reset();
        m_rotationController.reset();
    }

    public void tunePIDs() {
        m_translationXController.setPID(Mecanum2024Params.TranslationP, Mecanum2024Params.TranslationI, Mecanum2024Params.TranslationD);
        m_translationYController.setPID(Mecanum2024Params.TranslationP, Mecanum2024Params.TranslationI, Mecanum2024Params.TranslationD);
        m_rotationController.setPID(Mecanum2024Params.RotationP, Mecanum2024Params.RotationI, Mecanum2024Params.RotationD);

        m_translationXController.setTolerance(Mecanum2024Params.TranslationToleranceCentimeters);
        m_translationYController.setTolerance(Mecanum2024Params.TranslationToleranceCentimeters);
        m_rotationController.setTolerance(Mecanum2024Params.RotationToleranceRad);
    }

    public void moveFieldRelativeForPID() {
        double vX = MathUtil.clamp(m_translationXController.calculate(m_robotPose.getX()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());
        double vY = MathUtil.clamp(m_translationYController.calculate(m_robotPose.getY()),
                -m_mecanumConfigs.getMaxRobotSpeedMps(),
                m_mecanumConfigs.getMaxRobotSpeedMps());

        // Do some angle wrapping to ensure the shortest path is taken to get to the rotation target
        double normalizedRotationRad = m_robotPose.getHeading();
        if(normalizedRotationRad < 0) {
            normalizedRotationRad = m_robotPose.getHeading() + 2 * Math.PI; // Normalize to [0, 2PI]
        }

        double vOmega = MathUtil.clamp(m_rotationController.calculate(normalizedRotationRad),
                -m_mecanumConfigs.getMaxRobotRotationRps(),
                m_mecanumConfigs.getMaxRobotRotationRps());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vY, -vX, vOmega, getHeading()); // Transform the x and y coordinates to account for differences between global field coordinates and driver field coordinates
        move(speeds);
    }

    public void stop() {
        m_frontLeft.stopMotor();
        m_frontRight.stopMotor();
        m_backLeft.stopMotor();
        m_backRight.stopMotor();
    }

    @Override
    public void resetPose(Pose2d pose) {
        m_robotPose = pose;
    }

    @Override
    public Pose2d getPose() {
        return m_robotPose;
    }

    @Override
    public void periodic() {
        tunePIDs();
        m_odo.updatePose();
        m_odo.getPose().getRotation().times(-1); // Odometry heading is measured clockwise while we use counterclockwise rotations everywhere else, so we have to invert it here

        double currentAngleRad = m_initialAngleRad + m_odo.getPose().getHeading(); // Initial + Heading
        m_robotPose = new Pose2d(m_odo.getPose().getY(), m_odo.getPose().getX(), new Rotation2d(currentAngleRad));

        TelemetryPacket p = new TelemetryPacket();
        p.put("odo X", m_robotPose.getX());
        p.put("odo Y", m_robotPose.getY());
        p.put("odo Heading", Math.toDegrees(m_robotPose.getHeading()));
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}