package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Intake;

public class Spit extends CommandBase {
    private Intake m_intake;

    public Spit(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setVelocity(-0.1); // TODO tune this to be slow and out of the robot
    }
}
