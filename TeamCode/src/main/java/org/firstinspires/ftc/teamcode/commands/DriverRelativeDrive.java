package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

public class DriverRelativeDrive extends CommandBase {
    private Mecanum2024 m_mecanum;
    private CommandGamepad m_driver;

    public DriverRelativeDrive(Mecanum2024 mecanum, CommandGamepad driver) {
        m_mecanum = mecanum;
        m_driver = driver;
        addRequirements(m_mecanum, m_driver);
    }

    @Override
    public void execute() {
        m_mecanum.moveFieldRelative(m_driver.getLeftSlewedY(), m_driver.getLeftSlewedX(), -m_driver.getRightSlewedX());

    }
}
