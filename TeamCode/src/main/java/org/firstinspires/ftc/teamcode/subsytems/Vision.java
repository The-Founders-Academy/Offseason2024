package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.RedBlobDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Optional;

public class Vision extends SubsystemBase {
    private OpenCvCamera m_front;
    private RedBlobDetectionPipeline m_teamPropPipeline;
    private Mode m_mode = Mode.PROP;

    public enum Mode {
        PROP, TAG, STACK
    }

    public enum PropZone {
        LEFT, CENTER, RIGHT
    }

    public Vision(HardwareMap hardwareMap) {
        m_front = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontCamera"));

        m_front.setPipeline(m_teamPropPipeline);
    }

    /**
     * Returns the location of the team prop. Only works when in PROP mode.
     * @return The location (left, center, right) of the team prop.
     */
    public Optional<PropZone> getTeamPropLocation() {
        if(m_mode != Mode.PROP) return Optional.empty();
        return Optional.of(m_teamPropPipeline.getPropZone());
    }
}
