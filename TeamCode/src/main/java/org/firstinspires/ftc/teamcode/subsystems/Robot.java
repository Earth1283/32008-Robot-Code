package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

public class Robot {
    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public GoBildaPinpointDriver pinPoint;

    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }
    public void autoInit(HardwareMap hardwareMap) {
        exec = Executors.newScheduledThreadPool(5);
        intake.init(hardwareMap);
        shooter.init(hardwareMap);
    }

//    public void shoot3() {
//        intake.intakeIn();
//        intake.transToShooter();
////        exec.schedule(()-> intake.intakeIn(), 500, java.util.concurrent.TimeUnit.MILLISECONDS);
//        exec.schedule(()-> intake.transFromLeft(), 750, java.util.concurrent.TimeUnit.MILLISECONDS);
////        exec.schedule(()-> velocityCorrection = -50, 800, java.util.concurrent.TimeUnit.MILLISECONDS);
////        exec.schedule(()-> velocityCorrection = -70, 1550, java.util.concurrent.TimeUnit.MILLISECONDS);
//    }
//
//    public void intake3() {
//        intake.intakeIn();
//        intake.storeToLeft();
//        exec.schedule(()-> intake.servoStop(), 1500, java.util.concurrent.TimeUnit.MILLISECONDS);
//        exec.schedule(()-> intake.intakeStop(), 1900, java.util.concurrent.TimeUnit.MILLISECONDS);
//    }
}
