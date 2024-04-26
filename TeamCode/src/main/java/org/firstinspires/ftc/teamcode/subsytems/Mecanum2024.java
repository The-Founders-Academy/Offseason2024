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
        public static PIDCoefficients m_translationCoefficients = new PIDCoefficients(0.1, 0, 0);
        public static PIDCoefficients m_rotationCoefficients = new PIDCoefficients(0.1, 0, 0);
        public static double m_translationToleranceCentimeters = 10.0;
        public static double m_rotationToleranceRad = 0.0175; // 3 deg
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
        Mecanum2024Params.m_translationCoefficients = m_mecanumConfigs.getTranslationPIDValues();
        Mecanum2024Params.m_rotationCoefficients = m_mecanumConfigs.getRotationPIDValues();

        // These zeroes are replaced with real values as soon as tunePIDs() gets called
        m_translationXController = new PIDController(0, 0, 0);
        m_translationYController = new PIDController(0, 0, 0);
        m_rotationController = new PIDController(0, 0, 0);

    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
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
        m_translationXController.setPID(Mecanum2024Params.m_translationCoefficients.p,
                Mecanum2024Params.m_translationCoefficients.i,
                Mecanum2024Params.m_translationCoefficients.d);
        m_translationYController.setPID(Mecanum2024Params.m_translationCoefficients.p,
                Mecanum2024Params.m_translationCoefficients.i,
                Mecanum2024Params.m_translationCoefficients.d);
        m_rotationController.setPID(Mecanum2024Params.m_rotationCoefficients.p,
                Mecanum2024Params.m_rotationCoefficients.i,
                Mecanum2024Params.m_rotationCoefficients.d);

        m_translationXController.setTolerance(Mecanum2024Params.m_translationToleranceCentimeters);
        m_translationYController.setTolerance(Mecanum2024Params.m_translationToleranceCentimeters);
        m_rotationController.setTolerance(Mecanum2024Params.m_rotationToleranceRad);
    }

    public void moveFieldRelativeForPID() {
        double vX = m_translationXController.calculate(m_robotPose.getX());
        double vY = m_translationYController.calculate(m_robotPose.getY());
        double vOmega = m_rotationController.calculate(m_robotPose.getHeading());

        double mag = Math.sqrt(Math.pow(vX, 2) + Math.pow(vY, 2));

        double normX = (vX / mag) * m_mecanumConfigs.getMaxRobotSpeedMps();
        double normY = (vY / mag) * m_mecanumConfigs.getMaxRobotSpeedMps();
        double normOmega = MathUtil.clamp(vOmega, -m_mecanumConfigs.getMaxRobotRotationRps(), m_mecanumConfigs.getMaxRobotRotationRps());

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(normX, normY, normOmega, getHeading());
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
        TelemetryPacket p = new TelemetryPacket();
        displayPositionOnField(p);
        p.put("Current X", m_robotPose.getX());
        p.put("Current Y", m_robotPose.getY());
        p.put("Current heading", m_robotPose.getHeading());
        p.put("Target X", m_targetPose.getX());
        p.put("Target Y", m_targetPose.getY());
        p.put("Target heading", m_targetPose.getHeading());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}