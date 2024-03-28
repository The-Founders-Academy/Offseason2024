package org.firstinspires.ftc.teamcode.subsytems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Vision extends SubsystemBase {
    private AprilTagProcessor m_processor;
    private VisionPortal m_visionPortal;

    public Vision(HardwareMap hardwareMap, String cameraName) {
        m_processor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagID(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();
        m_visionPortal = new VisionPortal.Builder()
                .addProcessors(m_processor)
                .setCamera(hardwareMap.get(WebcamName.class, "frontCamera"))
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .build();
    }

    public int seenID() {
        if(!m_processor.getDetections().isEmpty()) {
            return m_processor.getDetections().get(0).id;
        }
        return -1;
    }
}
