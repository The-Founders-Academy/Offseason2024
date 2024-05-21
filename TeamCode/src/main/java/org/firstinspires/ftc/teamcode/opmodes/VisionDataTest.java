package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.GetVisionData;
import org.firstinspires.ftc.teamcode.subsytems.Vision;

@TeleOp(name="VisionDataTest")
public class VisionDataTest extends CommandOpMode {
    private Vision m_vision;
    @Override
    public void initialize() {
        m_vision = new Vision(hardwareMap);
        m_vision.setDefaultCommand(new GetVisionData(m_vision));
    }

}
