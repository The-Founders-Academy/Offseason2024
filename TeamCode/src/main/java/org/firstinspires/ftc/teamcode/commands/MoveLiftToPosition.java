package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Lift;

public class MoveLiftToPosition extends CommandBase {
    private Lift m_lift;
    private Lift.Extension m_extension = null;
    private int m_ticks;

    public MoveLiftToPosition(Lift lift, int ticks) {
        m_ticks = ticks;
        m_lift = lift;
        addRequirements(m_lift);
    }

    public MoveLiftToPosition(Lift lift, Lift.Extension extension) {
        m_lift = lift;
        m_extension = extension;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        if(m_extension == null) {
            m_lift.setTargetExtension(m_ticks);
        } else {
            m_lift.setTargetExtension(m_extension);
        }
    }

    @Override
    public void execute() {
        m_lift.set(0.75);
    }

    @Override
    public boolean isFinished() {
        return m_lift.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_lift.set(0);
    }
}
