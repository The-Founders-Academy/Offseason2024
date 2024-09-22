package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="StraightLineTest", group="test")
public class StraightLineTest extends OpMode {
    MotorEx fL;
    MotorEx fR;
    MotorEx bL;
    MotorEx bR;
    @Override
    public void init() {
        fL = new MotorEx(hardwareMap, "fL");
        fR = new MotorEx(hardwareMap, "fR");
        bL = new MotorEx(hardwareMap, "bL");
        bR = new MotorEx(hardwareMap, "bR");
    }

    @Override
    public void loop() {
        if(gamepad1.y) {            // forward
            fL.set(-0.5);
            fR.set(0.5);
            bL.set(-0.5);
            bR.set(0.5);
        } else if(gamepad1.a) {     // backward
            fL.set(0.5);
            fR.set(-0.5);
            bL.set(0.5);
            bR.set(-0.5);

        } else if(gamepad1.x) {     // left
            fL.set(0.5);
            fR.set(0.5);
            bL.set(-0.5);
            bR.set(-0.5);

        } else if(gamepad1.b) {     // right
            fL.set(-0.5);
            fR.set(-0.5);
            bL.set(0.5);
            bR.set(0.5);

        } else {
            fL.stopMotor();
            fR.stopMotor();
            bL.stopMotor();
            bR.stopMotor();
        }
    }
}
