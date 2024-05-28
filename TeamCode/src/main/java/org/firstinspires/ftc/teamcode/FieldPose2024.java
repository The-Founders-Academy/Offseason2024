package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;

// TODO: Determine all of these poses
public class FieldPose2024 {
    public static Pose2d AutoRedFar = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    public static Pose2d AutoRedClose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    public static Pose2d AutoBlueFar = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    public static Pose2d AutoBlueClose = new Pose2d(0, 0, Rotation2d.fromDegrees(90));
    public static Pose2d RedBackdropLeft = new Pose2d();
    public static Pose2d RedBackdropCenter = new Pose2d();
    public static Pose2d RedBackdropRight = new Pose2d();
    public static Pose2d RedLeftSpikeMark = new Pose2d();
    public static Pose2d RedCenterSpikeMark = new Pose2d();
    public static Pose2d RedRightSpikeMark = new Pose2d();
    public static Pose2d BlueLeftSpikeMark = new Pose2d();
    public static Pose2d BlueCenterSpikeMark = new Pose2d();
    public static Pose2d BlueRightSpikeMark = new Pose2d();
    public static Pose2d RedBackstageInner = new Pose2d();
    public static Pose2d RedBackstageOuter = new Pose2d();
    public static Pose2d BlueBackstageInner = new Pose2d();
    public static Pose2d BlueBackstageOuter = new Pose2d();

    public static Pose2d getFieldPose2024(Pose2d redPose) {

    }
}
