package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mecanum.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.util.DriverStation;


public class Mecanum2024 extends BaseMecanumDrive {

    private Pose2d m_robotPose;
    private HolonomicOdometry m_odo;
    private Encoder left;
    private Encoder right;
    private Encoder horizontal;
    private IMU m_gyro;

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
        horizontal = m_backLeft.encoder.setDistancePerPulse(cm_per_tick);
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
    }

    @Override
    public Rotation2d getHeading() {
        return m_robotPose.getRotation();
    }

    @Override
    public void periodic() {
        m_odo.updatePose();
        m_robotPose = m_odo.getPose();
        DriverStation.getInstance().getTelemetry().clearAll();
        DriverStation.getInstance().getTelemetry().addData("Deadwheel heading", m_robotPose.getRotation().getDegrees());
        DriverStation.getInstance().getTelemetry().addData("Rad heading", Math.toDegrees(m_robotPose.getHeading()));
        DriverStation.getInstance().getTelemetry().addData("Deadwheel Y", m_robotPose.getY());
        DriverStation.getInstance().getTelemetry().update();
    }
}