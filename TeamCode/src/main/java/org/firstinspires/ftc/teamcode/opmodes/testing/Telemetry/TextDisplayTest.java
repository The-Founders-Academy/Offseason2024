package org.firstinspires.ftc.teamcode.opmodes.testing.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="HelloWorld")
public class TextDisplayTest extends OpMode {



    @Override
    public void init() {
        telemetry.addData("Hello", "world");

    }

    @Override
    public void loop() {
        double left_x = gamepad1.left_stick_x;

        if(gamepad1.a){
            telemetry.addData("Button A", "Pressed");
        }
        else {
            telemetry.addData("Button A", "Not Pressed");
        }

        if(left_x < -0.5){
            telemetry.addData("Left Stick", "is a large negative number");
        }
        else if(left_x < 0){
            telemetry.addData("Left Stick", "is a small negative number");
        }
        else if(left_x < 0.5){
            telemetry.addData("Left Stick", "is a small positive number");
        }
        else{
            telemetry.addData("Left stick", "is a large positive number");
        }

        telemetry.addData("Left Stick x : ",left_x);
    }
}
