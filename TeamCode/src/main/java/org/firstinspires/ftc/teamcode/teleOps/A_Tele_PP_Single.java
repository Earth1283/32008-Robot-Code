//package org.firstinspires.ftc.teamcode.teleOps;
//
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndH;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndX;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.autoEndY;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
//import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.HeadingInterpolator;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.Robot;
//
//import java.util.function.Supplier;
//
//@Configurable
//@TeleOp
//public class A_Tele_PP_Single extends OpMode {
//    private Follower follower;
//    public static Pose startingPose; //See ExampleAuto to understand how to use this
//    private boolean automatedDrive;
//    double targetATAN, turretCurrentHeading;
//    int turretTargetHeading = 0;
//    private Supplier<PathChain> pathChain;
//    private TelemetryManager telemetryM;
//    private boolean slowMode = false;
//    private double slowModeMultiplier = 0.5;
//    double targetX = 136.5, targetY = 6;
//    Robot robot = new Robot();
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//
//        targetX = teleOpTargetX;
//        targetY = teleOpTargetY;
//
//        robot.init(hardwareMap);
//
//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//        robot.drivetrain.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH, autoEndY, 144 - autoEndX, AngleUnit.RADIANS, autoEndH - Math.PI / 2.0));
//
//        turretCurrentHeading = follower.getHeading();
//        targetATAN = Math.toDegrees(Math.atan2((targetY - follower.getPose().getY()), (targetX - follower.getPose().getX())));
//        if (Math.abs(targetATAN - turretCurrentHeading) <= 90) {
//            turretTargetHeading = (int) -(targetATAN - turretCurrentHeading);
//        } else {
//            turretTargetHeading = 0;
//        }
//
//        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
//                .addPath(new Path(new BezierLine(follower::getPose,
//                        new Pose(follower.getPose().getX()+5, follower.getPose().getY()+5)
//                )))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
//                        Math.toRadians(45), 0.8))
//                .build();
//    }
//
//    @Override
//    public void start() {
//        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
//        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
//        //If you don't pass anything in, it uses the default (false)
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//        //Call this once per loop
//        follower.update();
//        telemetryM.update();
//
//        if (!automatedDrive) {
//            if (!slowMode) follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x,
//                    -gamepad1.right_stick_x,
//                    true // Robot Centric
//            );
//            else follower.setTeleOpDrive(
//                    -gamepad1.left_stick_y * slowModeMultiplier,
//                    -gamepad1.left_stick_x * slowModeMultiplier,
//                    -gamepad1.right_stick_x * slowModeMultiplier,
//                    true // Robot Centric
//            );
//        }
//
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//
//        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }
//
//        telemetryM.debug("position", follower.getPose());
//        telemetryM.debug("velocity", follower.getVelocity());
//        telemetryM.debug("automatedDrive", automatedDrive);
//    }
//}