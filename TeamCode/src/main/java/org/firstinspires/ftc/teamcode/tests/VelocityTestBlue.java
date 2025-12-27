//package org.firstinspires.ftc.teamcode.tests;
//
//
//import static org.firstinspires.ftc.teamcode.constants.robotConstants.AUTO_BLUE_AIM_X;
//import static org.firstinspires.ftc.teamcode.constants.robotConstants.AUTO_BLUE_AIM_Y;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//@TeleOp
//public class VelocityTestBlue extends LinearOpMode {
//    Robot robot = new Robot();
//    double ps = 1;
//    double speed = 1600;
//    boolean shooterOn = false;
//    boolean toLeft = false, toRight = false, turrutFollow = false;
//    double targetATAN, turretCurrentHeading;
//    int turretTargetHeading = 0;
//    int turretCorrection = 0;
//    double velocityCorrection = 0;
//    double distance;
//    double velocity = 0, position = 0.35;
//    boolean chasisAutoAim = false;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap);
//        waitForStart();
//        while (opModeIsActive()) {
//            robot.drivetrain.driveFieldOriented(gamepad1, chasisAutoAim);
//            // 根据pp设置目标角度
//            Pose2D current = robot.drivetrain.getPosition();
//            turretCurrentHeading = current.getHeading(AngleUnit.DEGREES);
//
////            targetATAN = Math.toDegrees(Math.atan2(AUTO_BLUE_AIM_Y - current.getY(DistanceUnit.INCH), AUTO_BLUE_AIM_X - current.getX(DistanceUnit.INCH)));
////            if (Math.abs(targetATAN - turretCurrentHeading) <= 80) {
////                turretTargetHeading = (int) (targetATAN - turretCurrentHeading);
////            } else {
////                turretTargetHeading = 0;
////            }
//            distance = Math.abs(Math.hypot(AUTO_BLUE_AIM_Y - current.getY(DistanceUnit.INCH), AUTO_BLUE_AIM_X - current.getX(DistanceUnit.INCH)));
////            robot.shooter.turretToDegree(turretTargetHeading + turretCorrection);
//
//            if (gamepad1.dpadUpWasPressed()) velocity += 50;
//            if (gamepad1.dpadDownWasPressed()) velocity -= 50;
//
//            if (gamepad1.y) position += 0.005;
//            if (gamepad1.a) position -= 0.005;
//
//            if (gamepad1.right_trigger > 0) {
//                robot.intake.intakeIn();
//                robot.intake.midUp();
//            }
//            else if (gamepad1.left_trigger > 0) {
//                robot.intake.intakeOut();
//            }
//            else if (gamepad1.right_bumper) {
//                robot.intake.transferToShooter();
//                robot.intake.intakeIn();
//            }
//            else if(gamepad1.left_bumper){
//                robot.intake.leftUp();
//                robot.intake.rightUp();
//            }
//            else {
//                robot.intake.intakeStop();
//                robot.intake.midStop();
//                robot.intake.servoStop();
//            }
//
//            robot.shooter.setShooterVelocity(velocity);
//            robot.shooter.panelTo(position);
//
//
//            telemetry.addData("distance",distance);
//            telemetry.addData("target velocity", velocity);
//            telemetry.addData("current velocity", robot.shooter.leftShooter.getVelocity());
//            telemetry.addData("position", position);
//            telemetry.addData("distance",distance);
//            telemetry.update();
//        }
//    }
//}
