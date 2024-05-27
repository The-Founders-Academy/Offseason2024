package org.firstinspires.ftc.teamcode.opmodes.testing.auto;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@Autonomous(name="RotateTest")
public class AutoRotate extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private Rotation2d m_intiialRotation = new Rotation2d(0);
    private Rotation2d m_targetRotation = new Rotation2d(Math.PI);
    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, new Pose2d(0, 0, m_intiialRotation));

        DriveToPosition rotate = new DriveToPosition(m_mecanumDrive, m_mecanumDrive.getPose());
        CommandScheduler.getInstance().schedule(rotate.withTimeout(2000));
    }
}
