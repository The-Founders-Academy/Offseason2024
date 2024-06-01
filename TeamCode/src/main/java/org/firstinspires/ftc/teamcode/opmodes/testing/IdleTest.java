package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Arm;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@TeleOp(name="IdleTest")
public class IdleTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private Arm m_arm;

    private Intake m_intake;
    @Override
    public void initialize() {
        m_mecanumDrive = new Mecanum2024(hardwareMap, new MecanumConfigs(), new Pose2d(0 ,0, Rotation2d.fromDegrees(0)));
        m_arm = new Arm(hardwareMap);
        m_intake = new Intake(hardwareMap);
    }
}
