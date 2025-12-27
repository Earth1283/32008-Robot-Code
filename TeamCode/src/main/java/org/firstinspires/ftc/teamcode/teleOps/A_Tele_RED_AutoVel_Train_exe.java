//package org.firstinspires.ftc.teamcode.teleOps;
//
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndH;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;
//
//import com.bylazar.telemetry.JoinedTelemetry;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//import java.util.concurrent.Executors;
//import java.util.concurrent.ScheduledExecutorService;
//import java.util.concurrent.TimeUnit;
//
//@TeleOp
//public class A_Tele_RED_AutoVel_Train_exe extends LinearOpMode {
//    Robot robot = new Robot();
//    double targetX = 140, targetY = 144-140;
//    double resetX = 136, resetY = 72;
//    int turretTargetHeading = 0;
//    double targetATAN, turretCurrentHeading;
//    double panelPos = 0.63, shooterVelocity = 1900;
//    boolean shooterOn = false;
//    boolean aimBlue = true;
//    double distance;
//    int turretCorrection = 0;
//    JoinedTelemetry joinedTele;
//    boolean toLeft = false, toRight = false;
//
//    double distanceOffset = 0;
//    int velo = 1500;
//    double velocityCorrection = 0;
//    ElapsedTime timer = new ElapsedTime();
//    ScheduledExecutorService exec = Executors.newScheduledThreadPool(50);
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        joinedTele = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
//
//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//        targetX = 66;
//        targetY = -66;
//
//        waitForStart();
//
//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//
//
//        while (opModeIsActive()) {
//            robot.drivetrain.driveTrainConstantOriented(gamepad1,  aimBlue);
//            // 根据pp设置目标角度
//            Pose2D current = robot.drivetrain.getPosition();
//            distance = Math.abs(Math.hypot(targetY - current.getY(DistanceUnit.INCH), targetX - current.getX(DistanceUnit.INCH)));
//
//            if (gamepad2.right_trigger > 0) {
//                robot.intake.intakeIn();
//
//                shooterOn = false;
//            }
//            else if (gamepad2.left_trigger > 0) {
//                robot.intake.intakeOut();
//            }
//            else if (gamepad2.right_bumper) {
//                robot.intake.transferToShooter();
//                robot.intake.intakeIn();
//            }
////            else if(gamepad2.left_bumper){
////                robot.intake.leftUp();
////                robot.intake.rightUp();
////            }
//            else if (gamepad2.yWasPressed()) {
//
//            }
//            else if (gamepad2.aWasPressed()){
//
//            }
//            else {
//
//            }
//            if (gamepad2.dpadUpWasPressed()) velocityCorrection += 50;
//            if (gamepad2.dpadDownWasPressed()) velocityCorrection -= 50;
//
//            if (gamepad1.leftBumperWasPressed()) {
//
//                exec.schedule(() ->  shooterOn = !shooterOn, 200, TimeUnit.MILLISECONDS);
//                timer.reset();
//            }
//            if (shooterOn) {
//                robot.shooter.setShooterByDis(distance, velocityCorrection);
//
//            }
//            else {
//                robot.shooter.shooterStop();
//            }
//
//
//            telemetry.addData("Current",robot.shooter.leftShooter.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.addData("Velocity",robot.shooter.getShooterVelocity());
//            telemetry.addData("turretCurrentHeading",turretCurrentHeading);
//
//            telemetry.addData("Math.abs(targetATAN - turretCurrentHeading)",Math.abs(targetATAN - turretCurrentHeading));
//            telemetry.addData("distance",distance);
//            telemetry.addData("targetATAN",targetATAN);
//
//            telemetry.addData("current posx",current.getX(DistanceUnit.INCH));
//            telemetry.addData("current posy",current.getY(DistanceUnit.INCH));
//            telemetry.addData("current heading",current.getHeading(AngleUnit.DEGREES));
//
//
//
//            joinedTele.update();
//            robot.drivetrain.pinPoint.update();
//        }
//        robot.shooter.shooterStop();
//    }
//
//
//}
