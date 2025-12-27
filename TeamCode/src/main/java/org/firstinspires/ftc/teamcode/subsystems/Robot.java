package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Robot {
    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    public Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public GoBildaPinpointDriver pinPoint;

    public void autoShoot3Far() {
        intake.transferToShooterUPFirst();
        exec.schedule(()-> intake.transferToShooterStop(), 80, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterUP(), 1500, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.servoTransIn(), 1250, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterUP(), 1550, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterStop(), 2500, java.util.concurrent.TimeUnit.MILLISECONDS);
    }

    public void autoShoot3Close() {
        intake.transferToShooterDown();
        exec.schedule(()-> intake.transferToShooterStop(), 50, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterDown(), 300, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterStop(), 300, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterDown(), 300, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.intakeOut(), 100, java.util.concurrent.TimeUnit.MILLISECONDS);
        exec.schedule(()-> intake.transferToShooterStop(), 1500, java.util.concurrent.TimeUnit.MILLISECONDS);
    }

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

    public void startShooter() {

    }


    public void autoIntake() {
        intake.leftTransOut();
        intake.rightTransOut();
        exec.schedule(() -> intake.servoStop(), 1600, TimeUnit.MILLISECONDS);
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
