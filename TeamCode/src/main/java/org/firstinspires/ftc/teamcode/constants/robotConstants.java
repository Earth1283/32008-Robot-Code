package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class robotConstants {

    public static double KP = 0.0;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0;

    public static double SHOOTER_PITCH_FAR = 0.73;//0.73
    public static double SHOOTER_PITCH_CLOSE = 0.6;
    public static double SHOOTER_PITCH_UPPER_LIMIT = 0.934;
    public static double SHOOTER_PITCH_LOWER_LIMIT = 0.078;

    public static double SHOOTER_VELOCITY_FAR = 0.0;
    public static double SHOOTER_VELOCITY_CLOSE = 0.0;

    public static double SHOOTER_KP = 20;//10.0;
    public static double SHOOTER_KI = 5.0;
    public static double SHOOTER_KD = 2.5;
    public static double SHOOTER_KF = 0.0;

    public static int SHOOTER_TURRUT_LEFT = -258;
    public static int SHOOTER_TURRUT_RIGHT = 258;

    public static double TURRET_FULL_RANGE_DEGREE = 180;
    public static double TURRET_FULL_RANGE_ENCODER = 516;

    public static double YAW_KP = 0.0;
    public static double YAW_KI = 0.0;
    public static double YAW_KD = 0.0;



    public static volatile Pose lastKnownPose = null;
    public static volatile double heading = 0;
    public static volatile long lastUpdate = 0;
}
