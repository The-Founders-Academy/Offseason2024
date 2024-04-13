package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultVisionCommand;
import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="TestingVision")
public class TestingVision extends OpMode {
    private Vision m_vision;
    @Override
    public void init() {
        DriverStation.getInstance().setTelemetry(new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry));
        m_vision = new Vision(hardwareMap, "frontCamera");
        m_vision.setDefaultCommand(new DefaultVisionCommand(m_vision));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
