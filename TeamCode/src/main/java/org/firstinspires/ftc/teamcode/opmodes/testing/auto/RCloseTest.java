package org.firstinspires.ftc.teamcode.opmodes.testing.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@Autonomous(name="Red Close Test", group="test")
public class RCloseTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, FieldPose2024.AutoRedClose);
        SequentialCommandGroup path = new SequentialCommandGroup(
                new DriveToPosition(m_mecanumDrive, FieldPose2024.RedLeftSpikeMarkClose).withTimeout(5000),
                new DriveToPosition(m_mecanumDrive, FieldPose2024.RedBackdropInner)
        );

        CommandScheduler.getInstance().schedule(path);
    }
}
