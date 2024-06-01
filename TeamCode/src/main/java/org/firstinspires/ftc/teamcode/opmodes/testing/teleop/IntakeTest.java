package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakePixel;
import org.firstinspires.ftc.teamcode.commands.Spit;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="IntakeTest", group="test")
public class IntakeTest extends OpMode {
    private Intake m_intake;
    private CommandGamepad m_operator;
    @Override
    public void init() {
        DriverStation.getInstance().setTelemetry(new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()));
        m_operator = new CommandGamepad(gamepad2, 0, 0);
        m_intake = new Intake(hardwareMap);
        m_operator.buttonA().whileHeld(new IntakePixel(m_intake));
        m_operator.buttonB().whileHeld(new Spit(m_intake));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
