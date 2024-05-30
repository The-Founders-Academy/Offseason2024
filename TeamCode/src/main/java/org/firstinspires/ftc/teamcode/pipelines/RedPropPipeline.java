package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.teamcode.Constants.VisionConstants;
public class RedPropPipeline extends PropDetectionPipeline {

    public RedPropPipeline() {
        super(VisionConstants.LowerRedHue, VisionConstants.UpperRedHue);
    }
}
