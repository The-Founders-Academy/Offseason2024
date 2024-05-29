package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision.PropZone;

import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

public class DriveToPosition extends CommandBase {
    private Mecanum2024 m_mecanumDrive;
    private Pose2d m_targetPose;

    public DriveToPosition(Mecanum2024 mecanumDrive, Pose2d targetPose) {
        m_mecanumDrive = mecanumDrive;
        m_targetPose = targetPose;

        addRequirements(m_mecanumDrive);
    }

    @Override
    public void initialize() {
        m_mecanumDrive.setTargetPose(m_targetPose);
    }

    @Override
    public void execute() {
        m_mecanumDrive.moveFieldRelativeForPID();
    }

    public boolean isFinished() {
        return m_mecanumDrive.atTargetPose();
    }

    public void end(boolean interrupted) {
        m_mecanumDrive.resetPIDs();
        m_mecanumDrive.stop();
    }
}
