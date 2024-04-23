package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="OdometryTest")
public class DeadWheelOdoTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;

    @Override
    public void initialize() {
        DriverStation.getInstance().setTelemetry(telemetry);
        m_mecanumDrive = new Mecanum2024(hardwareMap, new MecanumConfigs(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
}
