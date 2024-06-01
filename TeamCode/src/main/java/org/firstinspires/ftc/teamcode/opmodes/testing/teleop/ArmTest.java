package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.commands.arm.MoveShoulderToPosition;
import org.firstinspires.ftc.teamcode.commands.arm.OpenHolder;
import org.firstinspires.ftc.teamcode.commands.arm.RetractArm;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Arm;

@TeleOp(name="Arm Test", group="test")
public class ArmTest extends CommandOpMode {
    private CommandGamepad m_test;
    private Arm m_arm;
    @Override
    public void initialize() {
        m_test = new CommandGamepad(gamepad1, 1, 1);
        m_arm = new Arm(hardwareMap);

        m_test.dpadDown().whenPressed(new RetractArm(m_arm).withTimeout(3000));
        m_test.dpadRight().whenPressed(new MoveLiftToPosition(m_arm, Arm.Extension.LOW_SCORE));
        m_test.dpadUp().whenPressed(new MoveLiftToPosition(m_arm, Arm.Extension.HIGH_SCORE));
        m_test.buttonA().whenPressed(new MoveShoulderToPosition(m_arm, 0));
        m_test.buttonB().whenPressed(new MoveShoulderToPosition(m_arm, Arm.ArmParams2024.ShoulderStowed));
        m_test.buttonY().whenPressed(new MoveShoulderToPosition(m_arm, Arm.ArmParams2024.ShoulderScore));
        m_test.buttonX().whenPressed(new OpenHolder(m_arm));

    }
}
