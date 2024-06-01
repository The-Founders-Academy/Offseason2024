package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Arm;

public class MoveLiftToPosition extends CommandBase {
    private Arm m_arm;
    private int m_targetTicks;

    public MoveLiftToPosition(Arm arm, int position) {
        m_arm = arm;
        m_targetTicks = position;
    }

    public MoveLiftToPosition(Arm arm, Arm.Extension extension) {
        m_arm = arm;

        switch(extension) {
            case STOW:
                m_targetTicks = Arm.ArmParams2024.LiftStowedTicks;
                break;
            case LOW_SCORE:
                m_targetTicks = Arm.ArmParams2024.LowScoreTicks;
                break;
            case MID_SCORE:
                m_targetTicks = Arm.ArmParams2024.MidScoreTicks;
                break;
            case HIGH_SCORE:
                m_targetTicks = Arm.ArmParams2024.HighScoreTicks;
                break;
        }

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_arm.setLiftHeight(m_targetTicks);
    }

    @Override
    public void execute() {
        m_arm.moveLift(0.75);
    }

    @Override
    public boolean isFinished() {
        return m_arm.atLiftTarget();
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.moveLift(0);
    }
}
