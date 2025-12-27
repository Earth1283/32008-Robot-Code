package org.firstinspires.ftc.teamcode.teleOps;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@TeleOp
public class A_Tele_AutoVel_AutoRotate extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 140, targetY = 140;
    double resetX = 136, resetY = 72;
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;

    boolean shooterOn = false;
    boolean aimBlue = true;
    double distance;
    int turretCorrection = 0;
    JoinedTelemetry joinedTele;
    boolean toLeft = false, toRight = false;
    boolean chasisAutoAim = false;

    double distanceOffset = 0;
    int velo = 1500;
    double velocityCorrection = 1350;
    ElapsedTime timer = new ElapsedTime();
    double panelPos = .85;


    boolean intakeStateStop1 = false, intakeStateStop2 = false, intakeOut1State = false, intakeOut2State = false;

    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        targetX = teleOpTargetX;
        targetY = teleOpTargetY;
        waitForStart();
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));


        while (opModeIsActive()) {
            robot.drivetrain.driveConstantOriented(gamepad1, chasisAutoAim);
            // 根据pp设置目标角度
            Pose2D current = robot.drivetrain.getPosition();
            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));

            if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut();
                shooterOn = false;
                intakeStateStop1 = false;
                intakeOut1State = false;
            }
            else if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                intakeOut1State = false;
            }else{
                intakeStateStop1 = true;
                intakeOut1State = true;

            }

            if (gamepad2.right_bumper) {
                intakeStateStop2 = false;
                robot.intake.intakeIn();
                robot.intake.transferToShooterUP();
            }
            else if(gamepad2.right_trigger > 0){
                robot.intake.intakeOut();
                intakeOut2State = false;
                robot.intake.transferToShooterDown();
            }else{
                intakeStateStop2 = true;
                intakeOut2State =true;
                robot.intake.transferToShooterStop();
            }

            if(gamepad2.xWasPressed()){
                velocityCorrection=1850;
                panelPos=0.78;
            }

            if (gamepad2.bWasPressed()) {
                velocityCorrection=1350;
                panelPos=0.85;
            }

            if (gamepad2.dpadUpWasPressed()) velocityCorrection += 50;
            if (gamepad2.dpadDownWasPressed()) velocityCorrection -= 50;
            if (gamepad2.dpadLeftWasPressed()) panelPos -= 0.01;
            if (gamepad2.dpadRightWasPressed()) panelPos += 0.01;
            if (panelPos < 0)panelPos = 0;
            if (panelPos > 1)panelPos = 1;


            if (gamepad2.leftBumperWasPressed()) {
                exec.schedule(() ->  shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
                timer.reset();
            }
            if (shooterOn) {
                robot.shooter.setShooter(panelPos, velocityCorrection);
            }
            else {
                robot.shooter.shooterStop();
            }

            if (gamepad1.bWasPressed()) {
                chasisAutoAim = !chasisAutoAim;// start/stop chasis auto aim
                gamepad1.rumbleBlips(1);
            }

            if(intakeStateStop1 && intakeStateStop2 && intakeOut1State && intakeOut2State)robot.intake.intakeStop();

            telemetry.addData("Velocity",velocityCorrection);
            telemetry.addData("panelPos",panelPos);
            telemetry.addData("y",gamepad1.left_stick_y);
            telemetry.addData("x",gamepad1.left_stick_x);
            telemetry.addData("rx",gamepad1.right_stick_x);






            telemetry.addData("Current",robot.shooter.leftShooter.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Velocity",robot.shooter.getShooterVelocity());
            telemetry.addData("turretCurrentHeading",turretCurrentHeading);
            telemetry.addData("Math.abs(targetATAN - turretCurrentHeading)",Math.abs(targetATAN - turretCurrentHeading));
            telemetry.addData("distance",distance);
            telemetry.addData("targetATAN",targetATAN);
            telemetry.addData("current posx",current.getX(DistanceUnit.INCH));
            telemetry.addData("current posy",current.getY(DistanceUnit.INCH));
            telemetry.addData("current heading",current.getHeading(AngleUnit.DEGREES));
            telemetry.addData("y",gamepad1.left_stick_y);
            telemetry.addData("x",gamepad1.left_stick_x);
            telemetry.addData("rx",gamepad1.right_stick_x);


            telemetry.addData("tttt","tttt");

            joinedTele.update();
            robot.drivetrain.pinPoint.update();


        }
        robot.shooter.shooterStop();
    }


}
