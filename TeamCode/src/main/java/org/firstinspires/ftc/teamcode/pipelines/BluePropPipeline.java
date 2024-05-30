package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.teamcode.Constants.VisionConstants;

public class BluePropPipeline extends PropDetectionPipeline {
    public BluePropPipeline() {
        super(VisionConstants.LowerBlueHue, VisionConstants.UpperBlueHue);
    }
}
