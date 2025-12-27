package org.firstinspires.ftc.teamcode.auto;


import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_CLOSE_FIRE_DISTANCE;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_CLOSE_OPEN_GATE;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_CLOSE_OPEN_GATE_CONTROL_POINT;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_CLOSE_SHOOT_SECOND_ROW_CONTROL_POINT;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_FIRST_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_FIRST_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_SECOND_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_SECOND_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_THIRD_END;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_INTAKE_THIRD_START;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_PARK;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_SHOOT_FIRST_ROW;

import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_SHOOT_PRELOAD;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_SHOOT_SECOND_ROW;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_SHOOT_THIRD_ROW;
import static org.firstinspires.ftc.teamcode.constants.autoConstants.BLUE_ClOSE_START;

//import static org.firstinspires.ftc.teamcode.constants.autoConstants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndH;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndY;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;


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
public class Auto_BLUE_CLOSE_12 extends OpMode {
    private static FollowerSubsystem follower;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    Robot robot = new Robot();

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = new FollowerSubsystem(hardwareMap, telemetryM);
        follower.setStartingPose(BLUE_ClOSE_START.copy());
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

                        // 发射初始球
                        new InstantCommand(() -> robot.shooter.setShooterClose()),
                        // shoot preload
                        new DrivePointToPoint(follower, BLUE_ClOSE_START, new Pose(46, 101, Math.toRadians(143))).setHoldEnd(false),


                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(1000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        // intake first row
                         new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_FIRST_START).setHoldEnd(true),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_FIRST_END).setHoldEnd(false).setMaxPower(1),
                        new InstantCommand(()->robot.intake.intakeStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        new DriveCurrentToPoint(follower, BLUE_CLOSE_OPEN_GATE_CONTROL_POINT, BLUE_CLOSE_OPEN_GATE).setHoldEnd(false), // smash the gate
                        // new InstantCommand(()->robot.intake.intakeStop()),
                        new InstantCommand(()->robot.intake.servoStop()),


                        // shoot first row
                        new InstantCommand(() -> robot.shooter.setShooterClose()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_SHOOT_FIRST_ROW).setHoldEnd(false),

                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(1000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        // intake second row
                         new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_SECOND_START).setHoldEnd(true),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_SECOND_END).setHoldEnd(false).setMaxPower(1),
                        new InstantCommand(()->robot.intake.intakeStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        // shoot second row
                        new InstantCommand(() -> robot.shooter.setShooterClose()),
                        new DriveCurrentToPoint(follower, BLUE_CLOSE_SHOOT_SECOND_ROW_CONTROL_POINT, BLUE_ClOSE_SHOOT_SECOND_ROW).setHoldEnd(false),


                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(1000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),


                        // 吸第三条
                         new InstantCommand(()->robot.intake.intakeOut()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_THIRD_START).setHoldEnd(true),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.autoIntake()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_INTAKE_THIRD_END).setHoldEnd(false).setMaxPower(1),
                         new InstantCommand(()->robot.intake.intakeStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        // 发射第三条
                        new InstantCommand(() -> robot.shooter.setShooterClose()),
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_SHOOT_THIRD_ROW).setHoldEnd(false),

                        //发射
                        new InstantCommand(()->robot.autoShoot3Close()), // Constantly open Intake, without transfer it should be fine
                        new WaitCommand(1000),
                        new InstantCommand(()->robot.shooter.shooterStop()),
                        new InstantCommand(()->robot.intake.transferToShooterStop()),
                        new InstantCommand(()->robot.intake.servoStop()),

                        // park
                        new DriveCurrentToPoint(follower, BLUE_ClOSE_PARK),
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