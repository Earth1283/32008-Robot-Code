package org.firstinspires.ftc.teamcode.teleOps;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;

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
public class A_Tele_Blue_AutoVel_Train_exe extends LinearOpMode {
    Robot robot = new Robot();
    double targetX = 140, targetY = 140;
    double resetX = 136, resetY = 72;
    int turretTargetHeading = 0;
    double targetATAN, turretCurrentHeading;
    double panelPos = 0.235, shooterVelocity = 1450;
    boolean shooterOn = false;
    boolean aimBlue = true;
    double distance;
    int turretCorrection = 0;
    JoinedTelemetry joinedTele;
    boolean toLeft = false, toRight = false;

    double distanceOffset = 0;
//    double panelPos = 0.235;
    int velo = 1450;

    double velocityCorrection = 0;
    ElapsedTime timer = new ElapsedTime();
    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//        targetX = 66;
//        targetY = 66;

        waitForStart();

//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));


        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1,false);
            // 根据pp设置目标角度
//            Pose2D current = robot.drivetrain.getPosition();
//            distance = Math.abs(Math.hypot(targetX - current.getY(DistanceUnit.INCH), targetY - current.getX(DistanceUnit.INCH)));

            if (gamepad2.right_trigger > 0) {
                robot.intake.intakeIn();
                robot.intake.leftTransIn();
                robot.intake.rightTransIn();
            }
            else if (gamepad2.left_trigger > 0) {
                robot.intake.intakeOut();
            }
            else {
                robot.intake.intakeStop();
                robot.intake.leftTransStop();
                robot.intake.rightTransStop();
            }

            if (gamepad2.right_bumper) {
                robot.intake.transferToShooterUP();
            }
            else if (gamepad2.left_bumper) {
                robot.intake.transferToShooterDown();
            }else{
                robot.intake.transferToShooterStop();
            }

            if (gamepad2.dpadUpWasPressed()) panelPos += 0.005;
            if (gamepad2.dpadDownWasPressed()) panelPos -= 0.005;
            if (gamepad2.dpadLeftWasPressed()) shooterVelocity += 50;
            if (gamepad2.dpadRightWasPressed()) shooterVelocity -= 50;

            if (gamepad1.leftBumperWasPressed()) {
                exec.schedule(() ->  shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
                timer.reset();
            }
            if (shooterOn) {
                robot.shooter.setShooter(panelPos,shooterVelocity);
            }
            else {
                robot.shooter.shooterStop();
            }


            telemetry.addData("Current",robot.shooter.leftShooter.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("Velocity",robot.shooter.getShooterVelocity());
            telemetry.addData("turretCurrentHeading",turretCurrentHeading);

            telemetry.addData("Math.abs(targetATAN - turretCurrentHeading)",Math.abs(targetATAN - turretCurrentHeading));
            telemetry.addData("distance",distance);
            telemetry.addData("targetATAN",targetATAN);

//            telemetry.addData("current posx",current.getX(DistanceUnit.INCH));
//            telemetry.addData("current posy",current.getY(DistanceUnit.INCH));
//            telemetry.addData("current heading",current.getHeading(AngleUnit.DEGREES));



            joinedTele.update();
//            robot.drivetrain.pinPoint.update();
        }
        robot.shooter.shooterStop();
    }


}
