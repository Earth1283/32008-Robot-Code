package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class RobotConstants {

    public static double TURRET_FULL_RANGE_DEGREE = 180;
    public static double TURRET_FULL_RANGE_ENCODER = 516;

    public static volatile double autoEndX = 72;
    public static volatile double autoEndY = 72;
    public static volatile double autoEndH = Math.PI / 2.0;

    // TeleOp Constants
    public static double DRIVE_SPEED = 0.8;
    public static double SHOOTER_VELOCITY_CLOSE = 1350;
    public static double SHOOTER_VELOCITY_FAR = 1850;
    public static double SHOOTER_PANEL_POS_CLOSE = 0.85;
    public static double SHOOTER_PANEL_POS_FAR = 0.78;

    public static volatile double teleOpTargetX = 136.5;
    public static volatile double teleOpTargetY = 138;

    public static double TELE_RED_START_X = -24;
    public static double TELE_RED_START_Y = 24;
    public static double TELE_RED_START_H = 90;
    public static double AUTO_BLUE_AIM_X = 138;
    public static double AUTO_BLUE_AIM_Y = 136.5;
    public static double AUTO_RED_AIM_X = 138;
    public static double AUTO_RED_AIM_Y = -136.5;




    public static volatile Pose lastKnownPose = null;
    public static volatile double heading = 0;
    public static volatile long lastUpdate = 0;
}
