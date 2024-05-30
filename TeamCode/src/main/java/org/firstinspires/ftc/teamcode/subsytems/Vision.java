package org.firstinspires.ftc.teamcode.subsytems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.auto.DetectTeamProp;
import org.firstinspires.ftc.teamcode.pipelines.BluePropPipeline;
import org.firstinspires.ftc.teamcode.pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.pipelines.RedPropPipeline;
import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private OpenCvCamera m_front;
    private PropDetectionPipeline m_propDetectionPipeline;

    private Mode m_mode = Mode.PROP;

    public enum Mode {
        PROP, TAG, STACK
    }

    public enum PropZone {
        LEFT, CENTER, RIGHT
    }

    public Vision(HardwareMap hardwareMap) {
        m_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontCamera"));
        switch(DriverStation.getInstance().getAlliance()) {
            case RED:
                m_propDetectionPipeline = new PropDetectionPipeline(Constants.VisionConstants.LowerRedHue, Constants.VisionConstants.UpperRedHue);
                break;
            case BLUE:
                m_propDetectionPipeline = new PropDetectionPipeline(Constants.VisionConstants.LowerBlueHue, Constants.VisionConstants.UpperBlueHue);
                break;

        }
        m_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontCamera"));
        m_front.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                m_front.startStreaming(1280, 720);
                m_front.setPipeline(m_propDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    /**
     * Returns the location of the team prop. Only works when in PROP mode.
     * @return The location (left, center, right) of the team prop.
     */
    public Optional<PropZone> getTeamPropLocation() {
        if (m_mode != Mode.PROP) return Optional.empty();
        return Optional.ofNullable(m_propDetectionPipeline.getPropZone());
    }

    @Override
    public void periodic() {
        TelemetryPacket p = new TelemetryPacket();
        p.put("Current PropZone", m_propDetectionPipeline.getPropZone());
        p.put("Pipeline tim MS", m_front.getPipelineTimeMs());
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
