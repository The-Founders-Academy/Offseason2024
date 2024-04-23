package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;

public class DefaultVisionCommand extends CommandBase {
    private Vision m_vision;


    public DefaultVisionCommand(Vision vision) {
        m_vision = vision;
        addRequirements(m_vision);
    }

    @Override
    public void execute() {
        DriverStation.getInstance().getTelemetry().addData("seen ID", m_vision.seenID());
        DriverStation.getInstance().getTelemetry().update();
    }
}
