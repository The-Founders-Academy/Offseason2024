package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Arm;

public class RetractArm extends CommandBase {
    private Arm m_arm;

    public RetractArm(Arm arm) {
        m_arm = arm;
    }

    @Override
    public void execute() {
        m_arm.setShoulderPosition(Arm.ArmParams2024.ShoulderStowed);
        m_arm.setLiftHeight(Arm.ArmParams2024.LiftStowedTicks);
        m_arm.moveLift(0.3);
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
