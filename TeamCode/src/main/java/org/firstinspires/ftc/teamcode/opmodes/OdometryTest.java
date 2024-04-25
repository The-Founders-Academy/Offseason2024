package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

public class OdometryTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;

    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs();
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
}
