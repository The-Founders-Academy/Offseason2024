package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

public class DriveToPositionWithTeamProp extends CommandBase {
    private Mecanum2024 m_mecanumDrive;
    private boolean m_goToBackdrop = false;

    public DriveToPositionWithTeamProp(Mecanum2024 mecanumDrive, boolean goToBackdrop) {
        m_mecanumDrive = mecanumDrive;
        m_goToBackdrop = goToBackdrop;

        addRequirements(m_mecanumDrive);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = new Pose2d();
        if(m_goToBackdrop == true) {
            switch(m_mecanumDrive.getPropZone()) {
                case LEFT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropLeft : FieldPose2024.BlueBackdropLeft;
                    break;
                case CENTER:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropCenter : FieldPose2024.BlueBackdropCenter;
                    break;
                case RIGHT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropRight : FieldPose2024.BlueBackdropRight;
                    break;
            }
        } else {
            switch(m_mecanumDrive.getPropZone()) {
                case LEFT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedLeftSpikeMark : FieldPose2024.BlueLeftSpikeMark;
                    break;
                case CENTER:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedCenterSpikeMark : FieldPose2024.BlueCenterSpikeMark;
                    break;
                case RIGHT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedRightSpikeMark : FieldPose2024.BlueRightSpikeMark;
                    break;
            }
        }
        m_mecanumDrive.setTargetPose(targetPose);
    }

    @Override
    public void execute() {
        m_mecanumDrive.moveFieldRelativeForPID();
    }

    @Override
    public boolean isFinished() {
        return m_mecanumDrive.atTargetPose();
    }

    @Override
    public void end(boolean interrupted) {
        m_mecanumDrive.resetPIDs();
    }
}
