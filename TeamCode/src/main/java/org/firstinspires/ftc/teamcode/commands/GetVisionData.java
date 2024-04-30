package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsytems.Vision;
import org.firstinspires.ftc.teamcode.subsytems.VisionData;

import java.util.Optional;

public class GetVisionData extends CommandBase {
    private Vision m_vision;
    public GetVisionData(Vision vision)
    {
        m_vision = vision;
        addRequirements(m_vision);
    }

    public void initialize() {
        m_vision.setTarget(8);
    }


    @Override
    public void execute()
    {

        Optional<VisionData> dataOptional = m_vision.getData();
        if (dataOptional.isPresent()){
            VisionData data = dataOptional.get();
            FtcDashboard.getInstance().getTelemetry().addData("x", data.getX());
            FtcDashboard.getInstance().getTelemetry().addData("y", data.getY());
            FtcDashboard.getInstance().getTelemetry().addData("id", data.getID());
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }

}
