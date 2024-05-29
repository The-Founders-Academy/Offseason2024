package org.firstinspires.ftc.teamcode.opmodes.competition.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.commands.Spit;
import org.firstinspires.ftc.teamcode.commands.auto.DetectTeamProp;
import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.commands.auto.DriveToPositionWithTeamProp;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.util.DriverStation;

@Autonomous(name="Red Far Prop 2+0")
public class RFarPropYellow extends CommandOpMode {
    private Mecanum2024 m_mecnaumDrive;
    private Lift m_lift;
    private Vision m_vision;
    private Intake m_intake;

    @Override
    public void initialize() {
        DriverStation.getInstance().setAlliance(DriverStation.Alliance.RED);
        MecanumConfigs configs = new MecanumConfigs();
        m_mecnaumDrive = new Mecanum2024(hardwareMap, configs, FieldPose2024.AutoRedFar);
        m_lift = new Lift(hardwareMap, "leftLift", "rightLift");
        m_vision = new Vision(hardwareMap);
        m_intake = new Intake("intake", hardwareMap);
        // 1) Place purple pixel on team prop spike mark
        // 2) Place yellow pixel on backdrop as directed
        // 3) Park
        SequentialCommandGroup auto = new SequentialCommandGroup(
                new DetectTeamProp(m_mecnaumDrive, m_vision).withTimeout(2000),
                new DriveToPositionWithTeamProp(m_mecnaumDrive, false).withTimeout(4000),
                new Spit(m_intake).withTimeout(1500),
                new DriveToPositionWithTeamProp(m_mecnaumDrive, true).withTimeout(8000),
                new DriveToPosition(m_mecnaumDrive, FieldPose2024.RedBackstageInner).withTimeout(2000)
        );

        CommandScheduler.getInstance().schedule(auto);
    }
}
