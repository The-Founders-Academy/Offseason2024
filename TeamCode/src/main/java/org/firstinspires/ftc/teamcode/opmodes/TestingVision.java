package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="TestingVision")
public class TestingVision extends OpMode {
    private Vision m_vision;
    @Override
    public void init() {
        DriverStation.getInstance().setTelemetry(new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry));
        m_vision = new Vision(hardwareMap, "frontCamera");
    }

    @Override
    public void loop() {
        DriverStation.getInstance().getTelemetry().addData("Check", "Working");
    }
}
