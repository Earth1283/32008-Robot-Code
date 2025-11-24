package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_SHOOTER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    public void setShooterByDis(double distance) {
        if (distance < 120) {
            setShooter(f(0, 0.00003, -0.0094, 1.0088, distance), f(0, -0.0368, 13.042, 736.11, distance) + velocityCorrection);
        } else {
            setShooter(0.26, 1950);
        }
    }

    public static double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }

    public void panelTo(double pos) {
        panel.setPosition(pos);
    }

}
