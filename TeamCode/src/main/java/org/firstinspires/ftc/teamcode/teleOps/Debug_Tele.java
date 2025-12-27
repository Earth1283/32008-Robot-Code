package org.firstinspires.ftc.teamcode.teleOps;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Debug Tele", group="Debug")
public class Debug_Tele extends LinearOpMode {
    Robot robot = new Robot();
    double panelPos = .85;
    boolean shooterOn = false;
    JoinedTelemetry joinedTele;
    double velocityCorrection = 1350;
    ElapsedTime timer = new ElapsedTime();
    boolean intakeStateStop1 = false, intakeStateStop2 = false, intakeOut1State = false, intakeOut2State = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();

        robot.shooter.setShooterClose();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 0.8, true);

            // Intake Logic
            if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut();
                shooterOn = false;
                intakeStateStop1 = false;
                intakeOut1State = false;
            }
            else if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                intakeOut1State = false;
            } else {
                intakeStateStop1 = true;
                intakeOut1State = true;
            }

            if (gamepad2.right_bumper) {
                 intakeStateStop2 = false;
                 robot.intake.intakeIn();
                 robot.intake.transferToShooterUP();
            }
            else if (gamepad2.right_trigger > 0) {
                robot.intake.intakeOut();
                intakeOut2State = false;
                robot.intake.transferToShooterDown();
            } else {
                intakeStateStop2 = true;
                intakeOut2State = true;
                robot.intake.transferToShooterStop();
            }

            if (intakeStateStop1 && intakeStateStop2 && intakeOut1State && intakeOut2State) {
                robot.intake.intakeStop();
            }

            // Shooter Settings
            if (gamepad2.x) {
                robot.shooter.setShooterClose();
            }
            else if (gamepad2.y) {
                robot.shooter.setShooter80();
            }
            else if (gamepad2.b) {
                robot.shooter.setShooterFar();
            }

            if (gamepad2.xWasPressed()) {
                velocityCorrection = 1850;
                panelPos = 0.78;
            }

            if (gamepad2.bWasPressed()) {
                velocityCorrection = 1350;
                panelPos = 0.85;
            }

            // Manual Adjustments
            if (gamepad2.dpadUpWasPressed()) velocityCorrection += 50;
            if (gamepad2.dpadDownWasPressed()) velocityCorrection -= 50;
            if (gamepad2.dpadLeftWasPressed()) panelPos -= 0.01;
            if (gamepad2.dpadRightWasPressed()) panelPos += 0.01;
            
            if (panelPos < 0) panelPos = 0;
            if (panelPos > 1) panelPos = 1;

            if (gamepad2.leftBumperWasPressed()) {
                robot.schedule(() -> shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
                timer.reset();
            }

            if (shooterOn) {
                robot.shooter.setShooter(panelPos, velocityCorrection);
            } else {
                robot.shooter.shooterStop();
            }

            // --- ADVANCED TELEMETRY ---
            telemetry.addLine("=== SHOOTER DEBUG ===");
            telemetry.addData("Shooter ON", shooterOn);
            telemetry.addData("Target Velocity", velocityCorrection);
            telemetry.addData("Actual Velocity L", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("Actual Velocity R", robot.shooter.rightShooter.getVelocity());
            telemetry.addData("Panel Position", "%.3f", panelPos);

            telemetry.addLine("\n=== POSITION DEBUG ===");
            Pose2D pose = robot.drivetrain.getPosition();
            telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.2f°", pose.getHeading(AngleUnit.DEGREES));

            telemetry.addLine("\n=== DRIVE DEBUG ===");
            telemetry.addData("Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Stick X", gamepad1.left_stick_x);
            telemetry.addData("Stick RX", gamepad1.right_stick_x);

            joinedTele.update();
        }
        robot.shooter.shooterStop();
        robot.stop();
    }
}
