package org.firstinspires.ftc.teamcode.teleOps;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@TeleOp
public class A_Tele_Single extends LinearOpMode {
    Robot robot = new Robot();
    double panelPos = .85;
    boolean shooterOn = false;
    JoinedTelemetry joinedTele;
    double velocityCorrection = 1350;
    ElapsedTime timer = new ElapsedTime();
    boolean intakeStateStop1 = false, intakeStateStop2 = false, intakeOut1State = false, intakeOut2State = false;

    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        waitForStart();

        robot.shooter.setShooterClose();

        while (opModeIsActive()) {
            robot.drivetrain.drive(gamepad1, 0.8, true);

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

             if (gamepad1.right_bumper) {
                 intakeStateStop2 = false;
                 robot.intake.intakeIn();
                 robot.intake.transferToShooterUP();
            }
            else if(gamepad1.right_trigger > 0){
                robot.intake.intakeOut();
                intakeOut2State = false;
                robot.intake.transferToShooterDown();
            }else{
                intakeStateStop2 = true;
                intakeOut2State =true;
                robot.intake.transferToShooterStop();
             }

            if(gamepad1.x){
                robot.shooter.setShooterClose();
            }

            else if (gamepad1.y) {
                robot.shooter.setShooter80();
            }
            else if (gamepad1.b){
                robot.shooter.setShooterFar();
            }


            if (gamepad1.dpadUpWasPressed()) velocityCorrection += 50;
            if (gamepad1.dpadDownWasPressed()) velocityCorrection -= 50;
            if (gamepad1.dpadLeftWasPressed()) panelPos -= 0.01;
            if (gamepad1.dpadRightWasPressed()) panelPos += 0.01;
            if (panelPos < 0)panelPos = 0;
            if (panelPos > 1)panelPos = 1;


            if (gamepad1.leftBumperWasPressed()) {
                exec.schedule(() ->  shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
                timer.reset();
            }
            if (shooterOn) {
                robot.shooter.setShooter(panelPos, velocityCorrection);
            }
            else {
                robot.shooter.shooterStop();
            }

            if(intakeStateStop1 && intakeStateStop2 && intakeOut1State && intakeOut2State)robot.intake.intakeStop();

            telemetry.addData("Velocity",velocityCorrection);
            telemetry.addData("panelPos",panelPos);
            telemetry.addData("y",gamepad1.left_stick_y);
            telemetry.addData("x",gamepad1.left_stick_x);
            telemetry.addData("rx",gamepad1.right_stick_x);


            joinedTele.update();


        }
        robot.shooter.shooterStop();
    }


}
