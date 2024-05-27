package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakePixel;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="IntakeTest", group="test")
public class IntakeTest extends OpMode {
    private Intake m_Intake;
    private CommandGamepad m_operator;
    @Override
    public void init() {
        DriverStation.getInstance().setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        m_operator = new CommandGamepad(gamepad2, 0, 0);
        m_Intake = new Intake("intake", hardwareMap);
        m_Intake.setDefaultCommand(new IntakePixel(m_Intake, m_operator));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
