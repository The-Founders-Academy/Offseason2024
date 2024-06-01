package org.firstinspires.ftc.teamcode.opmodes.competition.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.FieldPose2024;
import org.firstinspires.ftc.teamcode.commands.DriveToPosition;
import org.firstinspires.ftc.teamcode.commands.arm.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.commands.arm.OpenHolder;
import org.firstinspires.ftc.teamcode.commands.arm.RetractArm;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Arm;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

public class GreedyRClose extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private Arm m_arm;
    @Override
    public void initialize() {
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, FieldPose2024.RedLeftSpikeMarkClose);
        m_arm = new Arm(hardwareMap);

        SequentialCommandGroup tasks = new SequentialCommandGroup(
                new DriveToPosition(m_mecanumDrive, FieldPose2024.RedBackdropOuter).withTimeout(3000),
                new MoveLiftToPosition(m_arm, Arm.ArmParams2024.HighScoreTicks).withTimeout(5000),
                new OpenHolder(m_arm).withTimeout(2000),
                new RetractArm(m_arm).withTimeout(5000),
                new DriveToPosition(m_mecanumDrive, FieldPose2024.RedBackstagePrepOuter).withTimeout(1000),
                new DriveToPosition(m_mecanumDrive, FieldPose2024.RedBackstageOuter)
        );
        CommandScheduler.getInstance().schedule(tasks);
    }
}
