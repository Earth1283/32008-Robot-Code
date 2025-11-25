package org.firstinspires.ftc.teamcode.teleOps;

import static org.firstinspires.ftc.teamcode.constants.robotConstants.SHOOTER_PITCH_CLOSE;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.SHOOTER_PITCH_FAR;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_DEGREE;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.TURRET_FULL_RANGE_ENCODER;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.robotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class A_Tele extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 140, targetY = 140;
    double resetX = 136, resetY = 72;
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;
    double panelPos = 0.63, shooterVelocity = 1500;
    boolean shooterOn = false;
    boolean aimBlue = true;
    double distance;
    int turretCorrection = 0;
    JoinedTelemetry joinedTele;
    boolean toLeft = false, toRight = false;

    double distanceOffset = 0;

    int velo = 1500;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();
        long currTime = System.currentTimeMillis();
        if(robotConstants.lastKnownPose==null || currTime-robotConstants.lastUpdate>30000)
            robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, resetX, resetY, AngleUnit.DEGREES, 0));
        else
            robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH,robotConstants.lastKnownPose.getX(),robotConstants.lastKnownPose.getY(),AngleUnit.DEGREES,robotConstants.heading));


        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1.0);

            if (gamepad2.right_trigger > 0) robot.intake.intakeIn();
            else if (gamepad2.left_trigger > 0) robot.intake.intakeOut();
            else robot.intake.intakeStop();



            if(gamepad2.dpadUpWasPressed()){
                shooterVelocity+=100;
                if(shooterVelocity>2200) shooterVelocity = 2200;

            }
            if(gamepad2.dpadDownWasPressed()){
                shooterVelocity-=100;
                if(shooterVelocity<1000) shooterVelocity = 1000;
            }
            if(gamepad2.dpadLeftWasPressed()){
                panelPos-=0.1;
                if(panelPos<SHOOTER_PITCH_CLOSE) panelPos = SHOOTER_PITCH_CLOSE;

            }
            if(gamepad2.dpadRightWasPressed()){
                panelPos+=0.1;
                if(panelPos>SHOOTER_PITCH_FAR) panelPos = SHOOTER_PITCH_FAR;
            }

            if (gamepad2.circleWasPressed())
                shooterOn = !shooterOn;
            if (shooterOn) {
                robot. shooter.setShooter(panelPos, shooterVelocity);

            }else {
                robot.shooter.shooterHold();
            }

            if (gamepad2.rightBumperWasPressed()) {
                robot.intake.transferToShooter();
            }
            if(gamepad2.leftBumperWasPressed()){
                robot.intake.servoStop();
            }

            robot.shooter.panelTo(panelPos);

            joinedTele.addData("velo", shooterVelocity);
//            joinedTele.addData("turretDegree", robot.shooter.getTurretPosition() * (TURRET_FULL_RANGE_DEGREE / TURRET_FULL_RANGE_ENCODER));
            joinedTele.addData("panel", panelPos);
            joinedTele.addData("shooterT", shooterVelocity);
            joinedTele.addData("shooterV", robot.shooter.getShooterVelocity());
            joinedTele.update();
            robot.drivetrain.pinPoint.update();
        }
        robot.shooter.shooterStop();
    }

    public void resetPP(){
        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, resetX, resetY, AngleUnit.DEGREES, 0));
    }

    public void setShooterByDis(double distance) {
        double panel = 0.182 + 0.00768 * distance - 0.0000218 * distance * distance;
        double velocity = 1032 + 3.86 * distance + 0.0201 * distance * distance;
        robot. shooter.setShooter(panel, velocity);
//        setShooter(f(0.0000009, -0.0003, 0.0268, -0.284, distance), f(0.0007, 0.1541, 18.009, 691.05, distance));
    }

    public static double f(double a, double b, double c, double d, double x) {
        return a * Math.pow(x, 3) + b * Math.pow(x, 2) + c * x + d;
    }
}
