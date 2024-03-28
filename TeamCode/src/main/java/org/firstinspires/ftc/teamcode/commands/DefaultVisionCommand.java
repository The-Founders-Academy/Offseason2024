package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;

public class DefaultVisionCommand extends CommandBase {
    private Vision m_vision;
    private Mecanum2024 m_mecanumDrive;

    public DefaultVisionCommand(Vision vision, Mecanum2024 mecanumDrive) {
        m_vision = vision;
        m_mecanumDrive = mecanumDrive;

        addRequirements(m_vision);
    }

    @Override
    public void execute() {
        DriverStation.getInstance().getTelemetry().addData("seen ID", m_vision.seenID());
    }
}
