package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.util.DriverStation;

public class IntakePixel extends CommandBase {
    private Intake m_intake;
    public IntakePixel(Intake intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }
    @Override
    public void execute() {
        m_intake.setVelocity(-0.75);
        DriverStation.getInstance().getTelemetry().addData("intake speed", m_intake.getVelocity());
        DriverStation.getInstance().getTelemetry().update();

    }
    @Override
    public void end(boolean interrupted) {
        m_intake.setVelocity(0);
    }

}
