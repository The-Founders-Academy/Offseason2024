package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Arm;

public class OpenHolder extends CommandBase {
    private Arm m_arm;

    public OpenHolder(Arm arm) {
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.setHolderPosition(0.1);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
