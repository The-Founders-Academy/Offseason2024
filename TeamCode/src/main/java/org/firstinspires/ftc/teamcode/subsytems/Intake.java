package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;

public class Intake extends SubsystemBase {
    private MotorEx m_spinner;
    public Intake(String name, HardwareMap hardwareMap) {
        m_spinner = new MotorEx(hardwareMap, name);
        m_spinner.setRunMode(Motor.RunMode.RawPower);
    }

    public void setVelocity(double velocity) {
        m_spinner.set(velocity);
    }

    public double getVelocity() {
        return m_spinner.getVelocity();
    }
}
