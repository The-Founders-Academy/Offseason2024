package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Lift;
import org.firstinspires.ftc.teamcode.subsytems.Lift.Extension;

@TeleOp(name="LiftTest", group="test")
public class LiftTest extends CommandOpMode {
    private Lift m_lift;
    private CommandGamepad m_operator;

    @Override
    public void initialize() {
        m_lift = new Lift(hardwareMap, "leftLift", "rightLift");
        m_operator = new CommandGamepad(gamepad2, 1, 1);

        m_operator.buttonA().whenPressed(new MoveLiftToPosition(m_lift, Extension.STOW));
        m_operator.buttonB().whenPressed(new MoveLiftToPosition(m_lift, Extension.LOW_SCORE));
        m_operator.buttonX().whenPressed(new MoveLiftToPosition(m_lift, Extension.MID_SCORE));
        m_operator.buttonY().whenPressed(new MoveLiftToPosition(m_lift, Extension.HIGH_SCORE));
    }
}
