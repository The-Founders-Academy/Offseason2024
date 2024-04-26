package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name="MecanumPowerTest")
public class MecanumPowerTest extends CommandOpMode {
    private Mecanum2024 m_mecanum;
    private CommandGamepad m_driver;

    @Override
    public void initialize() {
        DriverStation.getInstance().setTelemetry(telemetry);
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanum = new Mecanum2024(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        m_driver = new CommandGamepad(gamepad1, 0.05, 0.05);
        m_mecanum.setDefaultCommand(new DriverRelativeDrive(m_mecanum, m_driver));
    }
}
