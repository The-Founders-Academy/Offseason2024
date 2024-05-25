package org.firstinspires.ftc.teamcode.opmodes.testing.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@Autonomous(name="DriveForwardTest", group="test")
public class DriveForwardTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private Pose2d m_initialPose = new Pose2d(0, 0, new Rotation2d(0));
    private double m_distanceCentimeters = 80;

    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, m_initialPose);

        double xDrive = m_distanceCentimeters * Math.cos(m_initialPose.getHeading());
        double yDrive = m_distanceCentimeters * Math.sin(m_initialPose.getHeading());
        Pose2d toPose = new Pose2d(xDrive, yDrive, m_initialPose.getRotation());

        DriveToPosition command = new DriveToPosition(m_mecanumDrive, toPose);
        CommandScheduler.getInstance().schedule(command);
    }
}
