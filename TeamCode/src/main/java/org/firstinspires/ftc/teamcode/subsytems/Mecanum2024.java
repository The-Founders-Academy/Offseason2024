package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Mecanum2024 extends BaseMecanumDrive {
    private HolonomicOdometry m_odo;

    private Pose2d m_robotPose;

    public Mecanum2024(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        super(hardwareMap, mecanumConfigs, initialPose);
        m_robotPose = initialPose;

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        OdoConfigs odoConfigs = new OdoConfigs()
                .deadWheelRadiusCentimeters(2.4)
                .ticksPerRevolution(2000.0)
                .trackWidthCentimeters(16.0)
                .perpPositionCentimeters(5.3);
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.toDegrees(m_robotPose.getHeading()));
    }

    @Override
    public void periodic() {
        m_odo.updatePose();
        m_robotPose = m_odo.getPose();
        DriverStation.getInstance().getTelemetry().clearAll();
        DriverStation.getInstance().getTelemetry().addData("Deadwheel heading", Math.toDegrees(m_robotPose.getHeading()));
        DriverStation.getInstance().getTelemetry().addData("Deadwheel X", m_robotPose.getX());
        DriverStation.getInstance().getTelemetry().addData("Deadwheel Y", m_robotPose.getY());
        DriverStation.getInstance().getTelemetry().update();
    }
}
