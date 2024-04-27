package org.firstinspires.ftc.teamcode.subsytems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.vision.AprilTagDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.ArrayList;
import java.util.Map;
import java.util.Optional;


public class Vision extends SubsystemBase {
   private AprilTagDetector m_detector;
   private Mode m_mode;

    public enum Mode {
        RED_BACKDROP,
        BLUE_BACKDROP,
        LOCALIZATION,
        STACK
    }

    public Vision(HardwareMap hardwareMap) {
        m_detector = new AprilTagDetector(hardwareMap);
        m_detector.WIDTH = Constants.VisionConstants.BackCameraWidth; // In Constants file
        m_detector.HEIGHT = Constants.VisionConstants.BackCameraHeight;
        m_detector.ORIENTATION = OpenCvCameraRotation.UPRIGHT;
        // m_detector.GPU_ENABLED = true; TODO Determine whether gpu enabled is good
        m_detector.init();


    }
    public Optional<VisionData> getData () {
        // sending Data to VisionData class
        Map<String, Integer> detection = m_detector.getDetection();
        if(detection != null) {
            return Optional.of(new VisionData(detection.get("x"), detection.get("y"), detection.get("id")));
        }
        return Optional.empty();
    }
    public void setTarget(@NonNull Integer... ids) {
        m_detector.setTargets(ids);
    }
}


