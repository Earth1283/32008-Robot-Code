package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class autoConstants {

    public static int AutoSpeedFar = 2000;
    public static int AutoSpeedClose = 1600;

    public static int PATTERN_ID = 21;
    public static Pose BLUE_FAR_START = new Pose(63, 8, Math.toRadians(90)); // x56
    public static Pose BLUE_FAR_SHOOT_PRELOAD = new Pose(60, 15, Math.toRadians(110));
    public static Pose BLUE_FAR_INTAKE_FAR_PREP = new Pose(40, 36, Math.toRadians(180));
    public static Pose BLUE_FAR_INTAKE_FAR_MID = new Pose(65, 36);
    public static Pose BLUE_FAR_INTAKE_FAR_FIRST_BALL = new Pose(35, 36, Math.toRadians(180));
    public static Pose BLUE_FAR_INTAKE_FAR_DONE = new Pose(19, 36, Math.toRadians(180));
    public static Pose BLUE_FAR_SHOOT_INTAKE = new Pose(63, 81, Math.toRadians(110));
    public static Pose BLUE_FAR_SHOOT_INTAKE_1 = new Pose(BLUE_FAR_SHOOT_INTAKE.getX(), BLUE_FAR_SHOOT_INTAKE.getY(), BLUE_FAR_SHOOT_INTAKE.getHeading());
    public static Pose BLUE_FAR_SHOOT_INTAKE_1_MID = new Pose(65, 42);
    public static Pose BLUE_FAR_INTAKE_MID_PREP = new Pose(40, 60, Math.toRadians(180));
    public static Pose BLUE_FAR_INTAKE_MID_MID = new Pose(65, 58);
    public static Pose BLUE_FAR_INTAKE_MID_DONE = new Pose(19, 60, Math.toRadians(180));
    public static Pose BLUE_FAR_SHOOT_INTAKE_2 = new Pose(BLUE_FAR_SHOOT_INTAKE.getX(), BLUE_FAR_SHOOT_INTAKE.getY(), BLUE_FAR_SHOOT_INTAKE.getHeading());
    public static Pose BLUE_FAR_PARK = new Pose(50, 68, Math.toRadians(0));


    public static Pose RED_FAR_START = new Pose(81, 8, Math.toRadians(90));
    public static Pose RED_FAR_SHOOT_PRELOAD = new Pose(86, 15, Math.toRadians(68));
    public static Pose RED_FAR_SHOOT_AFTER = new Pose(120, 8, Math.toRadians(68));
    public static Pose RED_FAR_INTAKE_FAR_PREP = new Pose(95, 36, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_FAR_MID = new Pose(79, 36); //曲线
    public static Pose RED_FAR_INTAKE_FAR_DONE = new Pose(125, 36, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT_INTAKE = new Pose(83, 15, Math.toRadians(70));
    public static Pose RED_FAR_SHOOT_INTAKE_1 = new Pose(81, 80, Math.toRadians(68));
    public static Pose RED_FAR_SHOOT_INTAKE_1_MID = new Pose(75, 42); //曲线
    public static Pose RED_FAR_INTAKE_MID_PREP = new Pose(95, 60, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_MID_MID = new Pose(104, 60); // 曲线
    public static Pose RED_FAR_INTAKE_MID_DONE = new Pose(125, 60, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT_INTAKE_2 = new Pose(104, 60, 0);
    public static Pose RED_FAR_SHOOT_INTAKE_2_MID = new Pose(65, 60); //曲线
    public static Pose RED_FAR_PARK = new Pose(94, 68, Math.toRadians(0));


    public static Pose BLUE_ClOSE_START = new Pose(64, 135, Math.toRadians(90));
    public static Pose BLUE_ClOSE_VISION_PRE = new Pose(64, 111, Math.toRadians(90));
    public static Pose BLUE_ClOSE_SHOOT_PRELOAD = new Pose(62, 90, Math.toRadians(135));
    public static Pose BLUE_ClOSE_INTAKE_CLOSE_PREP = new Pose(44, 84, Math.toRadians(190));
    public static Pose BLUE_ClOSE_INTAKE_CLOSE_DONE = new Pose(23, 84, Math.toRadians(190));
    public static Pose BLUE_ClOSE_SHOOT_INTAKE = new Pose(62, 90, Math.toRadians(135));
    public static Pose BLUE_ClOSE_SHOOT_INTAKE_1 = new Pose(BLUE_ClOSE_SHOOT_INTAKE.getX(), BLUE_ClOSE_SHOOT_INTAKE.getY(), BLUE_ClOSE_SHOOT_INTAKE.getHeading());
    public static Pose BLUE_ClOSE_SHOOT_INTAKE_1_MID = new Pose(45, 125); //曲线
    public static Pose BLUE_ClOSE_PARK = new Pose(45, 40, Math.toRadians(0));



    public static Pose RED_ClOSE_START = new Pose(112, 135.35, Math.toRadians(90));
    public static Pose RED_ClOSE_VISION_PRE = new Pose(88, 120, Math.toRadians(90));
    public static Pose RED_ClOSE_SHOOT_PRELOAD = new Pose(88, 87, Math.toRadians(45));
    public static Pose RED_ClOSE_INTAKE_CLOSE_PREP = new Pose(100, 84, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_CLOSE_DONE = new Pose(125, 84, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_MID_PREP = new Pose(100, 60, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_MID_DONE = new Pose(125, 60, Math.toRadians(0));
    public static Pose RED_ClOSE_SHOOT_INTAKE = new Pose(88, 88, Math.toRadians(45));
    public static Pose RED_ClOSE_SHOOT_INTAKE_1 = new Pose(RED_ClOSE_SHOOT_INTAKE.getX(), RED_ClOSE_SHOOT_INTAKE.getY(), RED_ClOSE_SHOOT_INTAKE.getHeading());
    public static Pose RED_ClOSE_SHOOT_INTAKE_1_MID = new Pose(99, 125); //曲线
    public static Pose RED_ClOSE_PARK = new Pose(94, 68, Math.toRadians(0));

}
