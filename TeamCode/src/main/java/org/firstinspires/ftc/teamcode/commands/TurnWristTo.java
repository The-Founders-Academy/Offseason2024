package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Wrist;

public class TurnWristTo extends CommandBase {
    private Wrist m_wrist;
    private double m_targetAngleDeg;

    public TurnWristTo(Wrist wrist, double angleDeg) {
        m_wrist = wrist;
        m_targetAngleDeg = angleDeg;
        addRequirements(m_wrist);
    }

    @Override
    public void execute() {
        m_wrist.setTarget(m_targetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return m_wrist.atTarget();
    }
}
