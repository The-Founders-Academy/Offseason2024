package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.MoveLiftToPosition;
import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.subsytems.Lift;

@TeleOp(name="LiftTest", group="test")
public class LiftTest extends CommandOpMode {
    private Lift m_lift;
    private CommandGamepad m_operator;

    @Override
    public void initialize() {
        m_lift = new Lift(hardwareMap, "leftLift", "rightLift");
        m_operator = new CommandGamepad(gamepad2, 1, 1);

        MoveLiftToPosition stowLift = new MoveLiftToPosition(m_lift, 150);
        MoveLiftToPosition test = new MoveLiftToPosition(m_lift, 1000);

        m_operator.buttonA().whenPressed(stowLift);
        m_operator.buttonB().whenPressed(test);
    }
}
