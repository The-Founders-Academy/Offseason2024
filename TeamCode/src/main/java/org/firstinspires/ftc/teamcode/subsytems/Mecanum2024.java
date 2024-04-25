package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
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

public class Mecanum2024 extends BaseMecanumDrive {
    IMU m_gyro;
    public Mecanum2024(HardwareMap hardwareMap, MecanumConfigs mecanumConfigs, Pose2d initialPose) {
        super(hardwareMap, mecanumConfigs, initialPose);

        m_frontLeft.setInverted(true);
        m_backLeft.setInverted(true);

        m_gyro = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        m_gyro.initialize(myIMUparameters);
    }

    @Override
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    @Override
    public void periodic() {
    }

    public void setWheelSpeeds(MecanumDriveWheelSpeeds speeds) {
        m_frontLeft.setVelocity(speeds.frontLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_frontRight.setVelocity(speeds.frontRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backLeft.setVelocity(speeds.rearLeftMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
        m_backRight.setVelocity(speeds.rearRightMetersPerSecond / m_mecanumConfigs.getMetersPerTick());
    }
}
