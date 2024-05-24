package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@TeleOp(name="DriveToPositionTest")
public class DriveToPositionTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private CommandGamepad m_driver;

    @Override
    public void initialize() {
        m_driver = new CommandGamepad(gamepad1, 0, 0);
        MecanumConfigs mecanumConfigs = new MecanumConfigs()
                .runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, mecanumConfigs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        m_driver.buttonA().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(15, 15, m_mecanumDrive.getHeading())).withTimeout(2000)); // Move 15 cm forward without changing heading
        m_driver.buttonX().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(m_mecanumDrive.getPose().getX(), m_mecanumDrive.getPose().getY(), Rotation2d.fromDegrees(90))).withTimeout(2000)); // return to starting position
        m_driver.buttonY().whenPressed(new DriveToPosition(m_mecanumDrive, new Pose2d(0, 0, Rotation2d.fromDegrees(90))).withTimeout(2000)); // Rotate to 90 deg
        m_driver.buttonB().whenPressed(new InstantCommand(() -> {
            m_mecanumDrive.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        }, m_mecanumDrive));
    }
}
