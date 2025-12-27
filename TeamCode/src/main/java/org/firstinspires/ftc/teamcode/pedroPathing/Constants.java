package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PIN_POINT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    // 跟随器常量由自动、PID和向心调谐器的值组成
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.1)
            .forwardZeroPowerAcceleration(-25.902412782495905)
            .lateralZeroPowerAcceleration(-59.922574362778484)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0.0, 0.0008, 0.6, 0.0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.13, 0.0, 0.015, 0.0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.7, 0.0, 0.08, 0.0))

            .centripetalScaling(0.0005);

    // 路径约束决定了路径在什么条件下可能结束
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);



    // 传动系统常量包含特定于您的传动系统类型的常量。例如，mecanum传动系统常数包含电机名称
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(78.8836564191683) //前后加速度
            .yVelocity(67.07177397960753) //左右加速度
            .rightFrontMotorName(RIGHT_FRONT)
            .rightRearMotorName(RIGHT_BACK)
            .leftRearMotorName(LEFT_BACK)
            .leftFrontMotorName(LEFT_FRONT)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true);

    // 本地化常量包含特定于您的本地化常量。例如，OTOS常量包括OTOS的硬件映射名称和偏移量
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.59) // 中心点左正
            .strafePodX(-1) //中心点前正
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(PIN_POINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

}