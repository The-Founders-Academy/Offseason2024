package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.util.DriverStation;

public class IntakePixel extends CommandBase {
    private Intake m_intake;
    private CommandGamepad m_operator;
    public IntakePixel(Intake intake, CommandGamepad operator) {
        m_intake = intake;
        m_operator = operator;
        addRequirements(m_intake, m_operator);
    }
    @Override
    public void execute() {
        m_intake.setVelocity(m_operator.getLeftY());
        DriverStation.getInstance().getTelemetry().addData("intake speed", m_intake.getVelocity());
        DriverStation.getInstance().getTelemetry().update();

    }
    @Override
    public void end(boolean interrupted) {
        m_intake.setVelocity(0);
    }

}
