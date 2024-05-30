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
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropInner : FieldPose2024.BlueBackdropOuter;
                    break;
                case CENTER:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropCenter : FieldPose2024.BlueBackdropCenter;
                    break;
                case RIGHT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedBackdropOuter : FieldPose2024.BlueBackdropInner;
                    break;
            }
        } else {
            switch(m_mecanumDrive.getPropZone()) {
                case LEFT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedLeftSpikeMarkFar : FieldPose2024.BlueLeftSpikeMarkFar;
                    break;
                case CENTER:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedCenterSpikeMarkFar : FieldPose2024.BlueCenterSpikeMarkFar;
                    break;
                case RIGHT:
                    targetPose = DriverStation.getInstance().getAlliance() == DriverStation.Alliance.RED ? FieldPose2024.RedRightSpikeMarkFar : FieldPose2024.BlueRightSpikeMarkFar;
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
