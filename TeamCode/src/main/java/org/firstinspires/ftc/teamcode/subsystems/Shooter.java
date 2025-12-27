package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.panelConstants.LEFT_KD;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.LEFT_KF;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.LEFT_KI;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.LEFT_KP;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.RIGHT_KD;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.RIGHT_KF;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.RIGHT_KI;
import static org.firstinspires.ftc.teamcode.constants.panelConstants.RIGHT_KP;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_SHOOTER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class Shooter {
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    public Servo panel;
    public static double velocityCorrection = 0;

    public void init (HardwareMap hardwareMap) {
        leftShooter = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);
        rightShooter = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        panel = hardwareMap.get(Servo.class, PANEL);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftShooter.setVelocityPIDFCoefficients(LEFT_KP, LEFT_KI, LEFT_KD, LEFT_KF);
        rightShooter.setVelocityPIDFCoefficients(RIGHT_KP, RIGHT_KI, RIGHT_KD, RIGHT_KF);
    }

    public void setShooterVelocity(double velocity) {
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(velocity);
    }
    public double getShooterVelocity() {
        return leftShooter.getVelocity();
    }

    public void shooterHold() {
        leftShooter.setVelocity(800);
        rightShooter.setVelocity(800);
    }

    public void shooterStop() {
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void setShooter(double panel, double velocity) {
        panelTo(panel);
        setShooterVelocity(velocity);
    }

    public void setShooterClose() {
        panelTo(0.8);
        setShooterVelocity(1400);
    }
    public void setShooter80() {
        panelTo(0.39);
        setShooterVelocity(2100);
    }
    public void setShooterFar() {
        panelTo(0.78);
        setShooterVelocity(1760);
    }

    public void setShooterByDis(double distance,double velocityCorrection) {
        if (distance < 120) {
            setShooter(f(-(1E-06), 0.0003, -0.0149,0.3097 , distance), f(-0.0003,0.0722,4.2811 ,1435.9 , distance) + velocityCorrection);
        } else {
            setShooter(0.83, 2550);
        }
    }


    public static double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public void panelTo(double pos) {
        panel.setPosition(pos);
    }

}
