package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class robotConstants {

    public static double TURRET_FULL_RANGE_DEGREE = 180;
    public static double TURRET_FULL_RANGE_ENCODER = 516;

    public static volatile double autoEndX = 72;
    public static volatile double autoEndY = 72;
    public static volatile double autoEndH = Math.PI / 2.0;

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
