package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="DriveTest")
public class DriveTest extends CommandOpMode {
    private CommandGamepad m_driver;
    private Mecanum2024 m_mecanumDrive;
    @Override
    public void initialize() {
        DriverStation.getInstance().setAlliance(DriverStation.Alliance.RED);
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(90)));
        m_driver = new CommandGamepad(gamepad1, 1, 1);
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_driver));
    }
}
