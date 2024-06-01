package org.firstinspires.ftc.teamcode.opmodes.competition.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.IntakePixel;
import org.firstinspires.ftc.teamcode.commands.Spit;
import org.firstinspires.ftc.teamcode.commands.arm.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.commands.arm.OpenHolder;
import org.firstinspires.ftc.teamcode.commands.arm.RetractArm;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Arm;
import org.firstinspires.ftc.teamcode.subsytems.Intake;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;

@TeleOp(name="main", group="competion")
public class Main extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private Intake m_intake;
    private Arm m_arm;
    private CommandGamepad m_driver;
    private CommandGamepad m_operator;
    @Override
    public void initialize() {
        m_driver = new CommandGamepad(gamepad1, 1, 1);
        m_operator = new CommandGamepad(gamepad2, 1, 1);
        MecanumConfigs configs = new MecanumConfigs().runMode(Motor.RunMode.RawPower);
        m_mecanumDrive = new Mecanum2024(hardwareMap, configs, new Pose2d(0, 0, Rotation2d.fromDegrees(Constants.MatchSingleton.EndAutoHeading)));
        m_intake = new Intake(hardwareMap);
        m_arm = new Arm(hardwareMap);

        driverControls();
        operatorControls();
    }

    public void driverControls() {
        m_mecanumDrive.setDefaultCommand(new DriverRelativeDrive(m_mecanumDrive, m_operator));
        m_driver.buttonB().whenPressed(new OpenHolder(m_arm));
    }

    public void operatorControls() {
        m_operator.buttonA().whenPressed(new IntakePixel(m_intake));
        m_operator.buttonB().whenPressed(new Spit(m_intake));

        m_operator.dpadDown().whenPressed(new RetractArm(m_arm));
        m_operator.dpadUp().whenPressed(new MoveLiftToPosition(m_arm, Arm.ArmParams2024.HighScoreTicks));
    }
}
