package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Lift;

public class MoveLiftToPosition extends CommandBase {
    private Lift m_lift;
    private int m_ticks;

    public MoveLiftToPosition(Lift lift, int ticks) {
        m_ticks = ticks;
        m_lift = lift;
        addRequirements(m_lift);
    }

    @Override
    public void initialize() {
        m_lift.setTargetExtension(m_ticks);
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
