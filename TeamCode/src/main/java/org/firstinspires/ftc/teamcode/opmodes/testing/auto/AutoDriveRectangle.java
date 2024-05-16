package org.firstinspires.ftc.teamcode.opmodes.testing.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@Autonomous(name="Simple rectangle PID")
public class AutoDriveRectangle extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    Pose2d m_start = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

    Pose2d m_topLeftCorner = new Pose2d(0, 15, Rotation2d.fromDegrees(90));
    Pose2d m_topRightCorner = new Pose2d(15, 15, Rotation2d.fromDegrees(90));
    Pose2d m_bottomRightCorner = new Pose2d(15, 0, Rotation2d.fromDegrees(90));
    @Override
    public void initialize() {
        DriverStation.getInstance().setTelemetry(telemetry);

        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, m_start);

        SequentialCommandGroup path = new SequentialCommandGroup();
        path.addCommands(
                new DriveToPosition(m_mecanumDrive, m_topLeftCorner).withTimeout(2000),
                new DriveToPosition(m_mecanumDrive, m_topRightCorner).withTimeout(2000),
                new DriveToPosition(m_mecanumDrive, m_bottomRightCorner).withTimeout(2000),
                new DriveToPosition(m_mecanumDrive, m_start).withTimeout(2000)
        );

        CommandScheduler.getInstance().schedule(path);
    }
}
