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

    public Mecanum2024(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        super(hardwareMap, mecanumConfigs, initialPose);
        m_robotPose = initialPose;

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

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
        horizontal.setDirection(Motor.Direction.REVERSE);

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


        m_odo.updatePose(initialPose);

        // These zeroes are replaced with real values as soon as tunePIDs() gets called
        m_translationXController = new PIDController(0, 0, 0);
        m_translationYController = new PIDController(0, 0, 0);
        m_rotationController = new PIDController(0, 0, 0);
    }

    @Override
    public Rotation2d getHeading() {
        return m_robotPose.getRotation();
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
        double vOmega = MathUtil.clamp(m_rotationController.calculate(m_robotPose.getHeading()),
                -m_mecanumConfigs.getMaxRobotRotationRps(),
                m_mecanumConfigs.getMaxRobotRotationRps());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, vOmega, getHeading());
        move(speeds);
    }

    public void displayPositionOnField(TelemetryPacket p) {
        p.fieldOverlay()
                .setFill("blue")
                .fillRect(centimetersToInches(-m_robotPose.getY()), centimetersToInches(m_robotPose.getX()), 40, 40);
    }

    // Dirty yankees
    private double centimetersToInches(double centimeters) {
        return centimeters * 2.54;
    }

    public void stop() {
        m_frontLeft.stopMotor();
        m_frontRight.stopMotor();
        m_backLeft.stopMotor();
        m_backRight.stopMotor();
    }

    public void resetPose2024(Pose2d pose) {
        m_robotPose = pose;
    }

    @Override
    public void periodic() {
        tunePIDs();
        m_odo.updatePose();
        m_robotPose = m_odo.getPose();
        m_robotPose.getRotation().times(-1); // Hold the door
        TelemetryPacket p = new TelemetryPacket();
        displayPositionOnField(p);
        p.put("Current X", m_robotPose.getX());
        p.put("Current Y", m_robotPose.getY());
        p.put("Current heading", m_robotPose.getHeading());
        p.put("IMU Heading", m_gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        p.put("Target X", m_targetPose.getX());
        p.put("Target Y", m_targetPose.getY());
        p.put("Target heading", m_targetPose.getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}