package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Arm;

public class MoveShoulderToPosition extends CommandBase {
    private Arm m_arm;
    private double m_targetPosition;

    public MoveShoulderToPosition(Arm arm, double targetPosition) {
        m_arm = arm;
        m_targetPosition = targetPosition;
        addRequirements(m_arm);
    }

    public MoveShoulderToPosition(Arm arm, Arm.ShoulderRotation position) {
        m_arm = arm;

        switch(position) {
            case HORIZONTAL:
                m_targetPosition = Arm.ArmParams2024.ShoulderStowed;
                break;
            case SCORE:
                m_targetPosition = Arm.ArmParams2024.ShoulderScore;
                break;
        }
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.setShoulderPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
