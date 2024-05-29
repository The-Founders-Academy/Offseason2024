package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.util.DriverStation;
import org.firstinspires.ftc.teamcode.util.DriverStation.Alliance;

// TODO: Determine all of these poses
public class FieldPose2024 {
    // Red poses
    public static Pose2d AutoBlueFar = new Pose2d(-103.01, 161.61, Rotation2d.fromDegrees(270));
    public static Pose2d AutoBlueClose = new Pose2d(39.19, 161.61, Rotation2d.fromDegrees(270));
    public static Pose2d BlueBackdropInner = new Pose2d(124.29, 107.49, Rotation2d.fromDegrees(90)); // TODO tune this
    public static Pose2d BlueBackdropCenter = new Pose2d(124.29, 92.94, Rotation2d.fromDegrees(90));
    public static Pose2d BlueBackdropOuter = new Pose2d(124.29, 76.89, Rotation2d.fromDegrees(90));
    public static Pose2d BlueLeftSpikeMark = new Pose2d(-82.49, 98.54, Rotation2d.fromDegrees(45)); // TODO tune this
    public static Pose2d BlueCenterSpikeMark = new Pose2d(-94.8, 85.85, Rotation2d.fromDegrees(270)); // TODO tune this
    public static Pose2d BlueRightSpikeMark = new Pose2d(-103.76, 102.27, Rotation2d.fromDegrees(-135)); // TODO tune this
    public static Pose2d BlueBackstageInner = new Pose2d(128.39, 40.69, Rotation2d.fromDegrees(0)); // TODO tune this
    public static Pose2d BlueBackstageOuter = new Pose2d(144.07, 161.61, Rotation2d.fromDegrees(0)); // TODO tune this

    // Blue poses are just mirrored across the center line
    public static Pose2d AutoRedFar = new Pose2d(AutoBlueFar.getX(), -AutoBlueFar.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d AutoRedClose = new Pose2d(AutoBlueClose.getX(), -AutoBlueFar.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedBackdropInner = new Pose2d(BlueBackdropInner.getX(), -BlueBackdropInner.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedBackdropCenter = new Pose2d(BlueBackdropCenter.getX(), -BlueBackdropCenter.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedBackdropOuter = new Pose2d(BlueBackdropOuter.getX(), -BlueBackdropOuter.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedLeftSpikeMark = new Pose2d(BlueRightSpikeMark.getX(), -BlueRightSpikeMark.getY(), Rotation2d.fromDegrees(135)); // TODO tune this
    public static Pose2d RedCenterSpikeMark = new Pose2d(BlueCenterSpikeMark.getX(), -BlueCenterSpikeMark.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedRightSpikeMark = new Pose2d(BlueRightSpikeMark.getX(), -BlueRightSpikeMark.getY(), Rotation2d.fromDegrees(45));
    public static Pose2d RedBackstageInner = new Pose2d(BlueBackstageInner.getX(), -BlueBackdropInner.getY(), Rotation2d.fromDegrees(90));
    public static Pose2d RedBackstageOuter = new Pose2d(BlueBackstageOuter.getX(), -BlueBackdropOuter.getY(), Rotation2d.fromDegrees(90));
}
