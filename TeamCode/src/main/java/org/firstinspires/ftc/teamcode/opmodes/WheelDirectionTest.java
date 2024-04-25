package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.CommandGamepad;
import org.firstinspires.ftc.teamcode.mecanum.MecanumConfigs;
import org.firstinspires.ftc.teamcode.subsytems.Mecanum2024;
import org.firstinspires.ftc.teamcode.util.DriverStation;

/**
 * This opmode can be used to confirm that each drive motor is driving in the right direction. The buttons on the driver controller correspond
 * to a wheel 45 degrees from it (e.g. X corresponds to frontLeft). All wheels should drive forward
 */
@TeleOp(name="DriveMotorDirectionTest")
public class WheelDirectionTest extends CommandOpMode {
    private Mecanum2024 m_mecanumDrive;
    private CommandGamepad m_driver;

    @Override
    public void initialize() {
        DriverStation.getInstance().setTelemetry(telemetry);
        m_mecanumDrive = new Mecanum2024(hardwareMap, new MecanumConfigs(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        m_driver = new CommandGamepad(gamepad1, 0, 0);
        m_mecanumDrive.setDefaultCommand(new RunCommand(() -> {
            MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds(
                    m_driver.buttonY().get() ? 1.0 : 0.0, // Front left
                    m_driver.buttonB().get() ? 1.0 : 0.0, // Front right
                    m_driver.buttonX().get() ? 1.0 : 0.0, // Back left
                    m_driver.buttonA().get() ? 1.0 : 0.0 // Back right

            );
            m_mecanumDrive.setWheelSpeeds(speeds);
        }, m_mecanumDrive, m_driver));
    }
}
