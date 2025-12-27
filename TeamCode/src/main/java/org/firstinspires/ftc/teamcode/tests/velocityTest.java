package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.TimeUnit;

@TeleOp
public class velocityTest extends LinearOpMode {
    Robot robot = new Robot();
    double panelPos = 0.5, velocity = 1500;
    boolean shooterOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 1, true);

            if (gamepad1.right_trigger > 0) {
                robot.intake.intakeIn();
                robot.intake.leftTransIn();
                robot.intake.rightTransIn();
            }
            else if (gamepad1.left_trigger > 0) {
                robot.intake.intakeOut();
            }
            else {
                robot.intake.intakeStop();
                robot.intake.leftTransStop();
                robot.intake.rightTransStop();
            }

            if (gamepad1.right_bumper) {
                robot.intake.transferToShooterUP();
            }
            else if (gamepad1.left_bumper) {
                robot.intake.transferToShooterDown();
            }else{
                robot.intake.transferToShooterStop();
            }

            if (gamepad1.dpadLeftWasPressed()) velocity -= 50;
            if (gamepad1.dpadRightWasPressed()) velocity += 50;
            if (gamepad1.dpad_up) panelPos += 0.005;
            if (gamepad1.dpad_down) panelPos -= 0.005;



            robot.shooter.panelTo(panelPos);
            robot.shooter.setShooterVelocity(velocity);

            telemetry.addData("Panel", panelPos);
            telemetry.addData("velocity target", velocity);
            telemetry.addData("velocity current l", robot.shooter.leftShooter.getVelocity());
            telemetry.addData("velocity current r", robot.shooter.rightShooter.getVelocity());
            telemetry.update();
        }
    }
}
