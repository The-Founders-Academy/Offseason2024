package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsytems.Intake;

public class IntakePixel extends CommandBase {
    private Intake m_intake;
    public IntakePixel(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_intake.setVelocity(1.0);
    }
    @Override
    public void end(boolean interrupted) {
        m_intake.setVelocity(0);
    }

}
