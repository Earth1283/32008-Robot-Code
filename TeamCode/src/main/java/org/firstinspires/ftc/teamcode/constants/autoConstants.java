package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.geometry.Pose;

public class autoConstants {


    public static double BLUE_CLOSE_FIRE_DISTANCE = 58;
    public static double BLUE_FAR_FIRE_DISTANCE = 134.33;
    public static double BLUE_INTAKE_START_X = 60;
    public static double BLUE_INTAKE_FIRST_ROW_X = 14;
    public static double BLUE_INTAKE_SECOND_ROW_X = 9;
    public static double BLUE_INTAKE_THIRD_ROW_X = 9;

    public static double BLUE_INTAKE_FIRST_ROW_Y = 84;
    public static double BLUE_INTAKE_SECOND_ROW_Y = 59;
    public static double BLUE_INTAKE_THIRD_ROW_Y = 36;

    public static Pose BLUE_FAR_START = new Pose(57, 8, Math.toRadians(90)); // x56
    public static Pose BLUE_FAR_SHOOT_PRELOAD = new Pose(60, 15, Math.toRadians(115));

    public static Pose BLUE_FAR_INTAKE_BESIDE_START = new Pose(34, 25, Math.toRadians(195));
    public static Pose BLUE_FAR_INTAKE_BESIDE_END = new Pose(14, 16, Math.toRadians(195));

    public static Pose BLUE_FAR_INTAKE_BESIDE_START2 = new Pose(34, 12, Math.toRadians(180));
    public static Pose BLUE_FAR_INTAKE_BESIDE_END2 = new Pose(14, 16, Math.toRadians(180));
    public static Pose BLUE_FAR_SHOOT_BESIDE = new Pose(BLUE_FAR_SHOOT_PRELOAD.getX(), BLUE_FAR_SHOOT_PRELOAD.getY(), Math.toRadians(115));

    public static Pose BLUE_FAR_PARK = new Pose(36, 12, Math.toRadians(180));


    public static Pose BLUE_ClOSE_START = new Pose(22, 124, Math.toRadians(143));
    public static Pose BLUE_ClOSE_SHOOT = new Pose(46, 103, Math.toRadians(140));
    public static Pose BLUE_ClOSE_SHOOT_PRELOAD = new Pose(BLUE_ClOSE_SHOOT.getX(), BLUE_ClOSE_SHOOT.getY(), BLUE_ClOSE_SHOOT.getHeading());
    public static Pose BLUE_ClOSE_INTAKE_FIRST_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_ClOSE_INTAKE_FIRST_END = new Pose(BLUE_INTAKE_FIRST_ROW_X, BLUE_INTAKE_FIRST_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_CLOSE_OPEN_GATE = new Pose(17, 70, Math.toRadians(180));
    public static Pose BLUE_CLOSE_OPEN_GATE_CONTROL_POINT =  new Pose(41, 78);
    public static Pose BLUE_ClOSE_SHOOT_FIRST_ROW = new Pose(BLUE_ClOSE_SHOOT.getX(), BLUE_ClOSE_SHOOT.getY(), BLUE_ClOSE_SHOOT.getHeading());
    public static Pose BLUE_ClOSE_INTAKE_SECOND_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_ClOSE_INTAKE_SECOND_END = new Pose(BLUE_INTAKE_SECOND_ROW_X, BLUE_INTAKE_SECOND_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_ClOSE_SHOOT_SECOND_ROW = new Pose(BLUE_ClOSE_SHOOT.getX(), BLUE_ClOSE_SHOOT.getY(), BLUE_ClOSE_SHOOT.getHeading());
    public static Pose BLUE_CLOSE_SHOOT_SECOND_ROW_CONTROL_POINT =  new Pose(30, 55);
    public static Pose BLUE_ClOSE_INTAKE_THIRD_START = new Pose(BLUE_INTAKE_START_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_ClOSE_INTAKE_THIRD_END = new Pose(BLUE_INTAKE_THIRD_ROW_X, BLUE_INTAKE_THIRD_ROW_Y, Math.toRadians(180));
    public static Pose BLUE_ClOSE_SHOOT_THIRD_ROW = new Pose(BLUE_ClOSE_SHOOT.getX(), BLUE_ClOSE_SHOOT.getY(), BLUE_ClOSE_SHOOT.getHeading());
    public static Pose BLUE_ClOSE_PARK = new Pose(25, 80, Math.toRadians(180));


    public static double RED_CLOSE_FIRE_DISTANCE = 58;
    public static double RED_INTAKE_START_X = 100;
    public static double RED_INTAKE_FIRST_ROW_X = 144 - BLUE_INTAKE_FIRST_ROW_X;
    public static double RED_INTAKE_SECOND_ROW_X = 144 - BLUE_INTAKE_SECOND_ROW_X;
    public static double RED_INTAKE_THIRD_ROW_X = 144 - BLUE_INTAKE_THIRD_ROW_X;
    public static double RED_INTAKE_FIRST_ROW_Y = BLUE_INTAKE_FIRST_ROW_Y;
    public static double RED_INTAKE_SECOND_ROW_Y = BLUE_INTAKE_SECOND_ROW_Y-3;
    public static double RED_INTAKE_THIRD_ROW_Y = BLUE_INTAKE_THIRD_ROW_Y-3;

    public static Pose RED_FAR_START = new Pose(144-BLUE_FAR_START.getX(), 8, Math.toRadians(90)); // x56
    public static Pose RED_FAR_SHOOT_PRELOAD = new Pose(144-BLUE_FAR_SHOOT_PRELOAD.getX(), 15, Math.toRadians(68));

    public static Pose RED_FAR_INTAKE_BESIDE_START = new Pose(144-BLUE_FAR_PARK.getX(), 16, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_BESIDE_END = new Pose(130, 16, Math.toRadians(0));

    public static Pose RED_FAR_INTAKE_BESIDE_START2 = new Pose(144-BLUE_FAR_PARK.getX(), 6, Math.toRadians(0));
    public static Pose RED_FAR_INTAKE_BESIDE_END2 = new Pose(130, 6, Math.toRadians(0));
    public static Pose RED_FAR_SHOOT_BESIDE = new Pose(RED_FAR_SHOOT_PRELOAD.getX(), RED_FAR_SHOOT_PRELOAD.getY(), Math.toRadians(65));



    public static Pose RED_FAR_PARK = new Pose(144-BLUE_FAR_PARK.getX(), 12, Math.toRadians(0));

    public static Pose RED_ClOSE_START = new Pose(144-BLUE_ClOSE_START.getX(), 124, Math.toRadians(37));
    public static Pose RED_ClOSE_SHOOT = new Pose(144-BLUE_ClOSE_SHOOT.getX(), 103, Math.toRadians(37));
    public static Pose RED_ClOSE_SHOOT_PRELOAD = new Pose(RED_ClOSE_SHOOT.getX(), RED_ClOSE_SHOOT.getY(), RED_ClOSE_SHOOT.getHeading());
    public static Pose RED_ClOSE_INTAKE_FIRST_START = new Pose(RED_INTAKE_START_X, RED_INTAKE_FIRST_ROW_Y, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_FIRST_END = new Pose(RED_INTAKE_FIRST_ROW_X, RED_INTAKE_FIRST_ROW_Y, Math.toRadians(0));
    public static Pose RED_CLOSE_OPEN_GATE = new Pose(144 - BLUE_CLOSE_OPEN_GATE.getX()+7, BLUE_CLOSE_OPEN_GATE.getY()+2, Math.toRadians(0));
    public static Pose RED_CLOSE_OPEN_GATE_CONTROL_POINT = new Pose(144 - BLUE_CLOSE_OPEN_GATE_CONTROL_POINT.getX(), 75);
    public static Pose RED_ClOSE_SHOOT_FIRST_ROW = new Pose(RED_ClOSE_SHOOT.getX(), RED_ClOSE_SHOOT.getY(), RED_ClOSE_SHOOT.getHeading());
    public static Pose RED_ClOSE_INTAKE_SECOND_START = new Pose(RED_INTAKE_START_X, RED_INTAKE_SECOND_ROW_Y, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_SECOND_END = new Pose(RED_INTAKE_SECOND_ROW_X, RED_INTAKE_SECOND_ROW_Y, Math.toRadians(0));
    public static Pose RED_ClOSE_SHOOT_SECOND_ROW = new Pose(RED_ClOSE_SHOOT.getX(), RED_ClOSE_SHOOT.getY(), RED_ClOSE_SHOOT.getHeading());
    public static Pose RED_CLOSE_SHOOT_SECOND_ROW_CONTROL_POINT =  new Pose(144 - BLUE_CLOSE_SHOOT_SECOND_ROW_CONTROL_POINT.getX(), 55);
    public static Pose RED_ClOSE_INTAKE_THIRD_START = new Pose(RED_INTAKE_START_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_ClOSE_INTAKE_THIRD_END = new Pose(RED_INTAKE_THIRD_ROW_X, RED_INTAKE_THIRD_ROW_Y, Math.toRadians(0));
    public static Pose RED_ClOSE_SHOOT_THIRD_ROW = new Pose(RED_ClOSE_SHOOT.getX(), RED_ClOSE_SHOOT.getY(), RED_ClOSE_SHOOT.getHeading());
    public static Pose RED_ClOSE_PARK = new Pose(144 - BLUE_ClOSE_PARK.getX(), 73, Math.toRadians(0));



}
