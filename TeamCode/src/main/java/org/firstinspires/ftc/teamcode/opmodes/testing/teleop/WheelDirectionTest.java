package org.firstinspires.ftc.teamcode.opmodes.testing.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="WheelDirectionTest", group="test")
public class WheelDirectionTest extends OpMode {
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
        if(gamepad1.x) {
            bL.set(0.5);
        } else if(gamepad1.y) {
            fL.set(0.5);
        } else if(gamepad1.b) {
            fR.set(0.5);
        } else if(gamepad1.a) {
            bR.set(0.5);
        } else {
            fL.stopMotor();
            fR.stopMotor();
            bL.stopMotor();
            bR.stopMotor();
        }
    }
}
