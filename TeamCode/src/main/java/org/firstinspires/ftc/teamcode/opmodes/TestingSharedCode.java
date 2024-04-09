package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.IntakePixel;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@TeleOp(name="TestingSharedCode")
public class TestingSharedCode extends OpMode {
    private Mecanum2024 m_mecanumDrive;
    private Intake m_intake;
    private CommandGamepad m_driver;
    @Override
    public void init() {

    }
    @Override
    public void loop() {

    }
}
