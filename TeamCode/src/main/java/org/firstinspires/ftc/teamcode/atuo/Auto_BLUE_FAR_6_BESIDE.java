package org.firstinspires.ftc.teamcode.atuo;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_INTAKE_BESIDE_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_INTAKE_BESIDE_END2;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_INTAKE_BESIDE_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_INTAKE_BESIDE_START2;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_PARK;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_SHOOT_BESIDE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_FAR_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_INTAKE_BESIDE_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_INTAKE_BESIDE_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_PARK;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_SHOOT_BESIDE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.RED_FAR_START;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.robotConstants.teleOpTargetY;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.DriveCurrentToPoint;
import org.firstinspires.ftc.teamcode.commands.DrivePointToPoint;
import org.firstinspires.ftc.teamcode.subsystems.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class Auto_BLUE_FAR_6_BESIDE extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    Robot robot = new Robot();



    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(BLUE_FAR_START.copy());
        robot.autoInit(hardwareMap);

        telemetryM.update();
        Drawing.init();
    }

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower.follower);
    }

    @Override
    public void init_loop() {
        follower.follower.update();

        drawOnlyCurrent();
    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        follower.follower.update();
        draw();
    }

    @Override
    public void start() {
        follower.follower.activateAllPIDFs();
        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.shooter.setShooterFar()),
                        new WaitCommand(1000),
                        new DrivePointToPoint(follower,BLUE_FAR_START, BLUE_FAR_SHOOT_PRELOAD),

                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(2000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),



                        new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_START),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_END).setMaxPower(1),
                        new WaitCommand(800),
                        new InstantCommand(()->robot.autoIntake()),
                        new InstantCommand(()->robot.intake.intakeStop()),

                        new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_START),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_END).setMaxPower(1),
                        new WaitCommand(800),
                        new InstantCommand(()->robot.autoIntake()),
                        new InstantCommand(()->robot.intake.intakeStop()),

                        new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_START),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, new Pose(10,10,Math.toRadians(195))).setMaxPower(1),
                        new WaitCommand(800),
                        new InstantCommand(()->robot.autoIntake()),
                        new InstantCommand(()->robot.intake.intakeStop()),


                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_BESIDE),
                        new InstantCommand(() -> robot.shooter.setShooterFar()),
                        new WaitCommand(1000),

                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(2000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),


                        //第二次吸取
                        new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_START2),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_FAR_INTAKE_BESIDE_END2).setMaxPower(1),
                        new WaitCommand(1200),
                        new InstantCommand(()->robot.intake.intakeStop()),


                        new DriveCurrentToPoint(follower, BLUE_FAR_SHOOT_BESIDE),
                        new InstantCommand(() -> robot.shooter.setShooterFar()),
                        new WaitCommand(1000),

                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(2000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),


                        // park
                        new DriveCurrentToPoint(follower, BLUE_FAR_PARK),
                        new WaitCommand(100),
                        new InstantCommand(this::stop)

                )
        );
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
        follower.breakFollowing();

        autoEndX = follower.follower.getPose().getX();
        autoEndY = follower.follower.getPose().getY();
        autoEndH = follower.follower.getPose().getHeading();

        teleOpTargetX = 136.5;
        teleOpTargetY = 138;
    }
}