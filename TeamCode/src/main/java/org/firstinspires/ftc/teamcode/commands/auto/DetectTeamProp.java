package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision;

public class DetectTeamProp extends CommandBase {
    private Mecanum2024 m_mecanumDrive;
    private Vision m_vision;

    public DetectTeamProp(Mecanum2024 mecanumDrive, Vision vision) {
        m_mecanumDrive = mecanumDrive;
        m_vision = vision;

        addRequirements(m_mecanumDrive, m_vision);
    }

    @Override
    public void execute() {
        if(m_vision.getTeamPropLocation().isPresent()) {
            m_mecanumDrive.setPropZone(m_vision.getTeamPropLocation().get());
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
