package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.commands.TurnWristTo;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.Wrist;
import org.firstinspires.ftc.teamcode.subsytems.Wrist.WristParams2024;

@TeleOp(name="FullArmTest", group="testing")
public class FullArmTest extends CommandOpMode {
    private CommandGamepad m_driver;
    private CommandGamepad m_operator;
    private Lift m_lift;
    private Wrist m_wrist;

    @Override
    public void initialize() {
        m_driver = new CommandGamepad(gamepad1, 1, 1);
        m_driver = new CommandGamepad(gamepad2, 1, 1);
        m_lift = new Lift(hardwareMap);
        m_wrist = new Wrist(hardwareMap);

        m_operator.buttonA().whenPressed(new MoveLiftToPosition(m_lift, Lift.Extension.STOW));
        m_operator.buttonB().whenPressed(new MoveLiftToPosition(m_lift, Lift.Extension.LOW_SCORE));
        m_operator.buttonX().whenPressed(new MoveLiftToPosition(m_lift, Lift.Extension.MID_SCORE));
        m_operator.buttonY().whenPressed(new MoveLiftToPosition(m_lift, Lift.Extension.HIGH_SCORE));

        m_driver.buttonA().whenPressed(new TurnWristTo(m_wrist, WristParams2024.SCORE_ANGLE));
        m_driver.buttonB().whenPressed(new TurnWristTo(m_wrist, WristParams2024.STOW_ANGLE));
    }
}
