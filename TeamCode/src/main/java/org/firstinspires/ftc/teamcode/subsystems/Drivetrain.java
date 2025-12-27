package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PIN_POINT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;

import static org.firstinspires.ftc.teamcode.constants.RobotConstants.AUTO_BLUE_AIM_X;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.AUTO_BLUE_AIM_Y;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.AUTO_RED_AIM_X;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.AUTO_RED_AIM_Y;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetX;
import static org.firstinspires.ftc.teamcode.constants.RobotConstants.teleOpTargetY;

import com.bylazar.gamepad.GamepadManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Drivetrain {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    public GoBildaPinpointDriver pinPoint;
    private double theta, power, turn, realTheta;

    // PID coefficients for heading control
    private final double headingKp = 0.08;//0.06
    private final double headingKd = 0.004;//0.003

    // PID state variables
    private double headingLastError = 0.0;
    private long headingLastTime = 0;
    double fixedFieldHeading = 0.0;

    public void init (HardwareMap hardwareMap) {
        pinPoint = hardwareMap.get(GoBildaPinpointDriver.class, PIN_POINT);
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT);
        leftBack = hardwareMap.get(DcMotorEx.class, LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT);
        rightBack = hardwareMap.get(DcMotorEx.class, RIGHT_BACK);

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

//        PIDFCoefficients coeffs = new PIDFCoefficients(50.0, 0.05, 1.0, 12.0);
//
//
//        leftFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
//        leftBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
//        rightFront.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
//        rightBack.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coeffs);
//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // strafex 后负
        // forwardy 左负
        pinPoint.setOffsets(10,5.9 , DistanceUnit.CM);
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinPoint.resetPosAndIMU();

    }

    public void drive(Gamepad gamepad, double powerScale, boolean isBlue) {
        double y, x, rx;
        y = -gamepad.left_stick_y;
        x = gamepad.left_stick_x;
        rx = gamepad.right_stick_x * 0.85;
        y = Math.abs(y)<=0.1 ? 0 : y;
        x = Math.abs(x)<=0.1 ? 0 : x;
        rx = Math.abs(rx)<=0.1 ? 0 : rx;

        leftFront.setPower((y + x + rx) * powerScale);
        leftBack.setPower((y - x + rx) * powerScale);
        rightFront.setPower((y - x - rx) * powerScale);
        rightBack.setPower((y + x - rx) * powerScale);
    }

    public void drive(GamepadManager gamepad, double powerScale) {
        double y = -gamepad.getLeftStickY(), x = gamepad.getLeftStickX(), rx = gamepad.getRightStickX() * 0.85;
        leftFront.setPower((y + x + rx) * powerScale);
        leftBack.setPower((y - x + rx) * powerScale);
        rightFront.setPower((y - x - rx) * powerScale);
        rightBack.setPower((y + x - rx) * powerScale);
    }

    public double getHeading() {
        return pinPoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    public void driveFieldOriented(Gamepad gamepad, boolean chasisAutoAim) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 0.85;
        y = Math.abs(y)<=0.1 ? 0 : y;
        x = Math.abs(x)<=0.1 ? 0 : x;
        rx = Math.abs(rx)<=0.1 ? 0 : rx;
        pinPoint.update();

        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);
        turn = rx; // Fix without testing: autoaim now stopped blocking rightstick

        if (chasisAutoAim)
        {
            Pose2D current = pinPoint.getPosition();
            double currentHeading, targetHeading, headingError;
            currentHeading = current.getHeading(AngleUnit.DEGREES);
            targetHeading = Math.toDegrees(Math.atan2(66 - current.getY(DistanceUnit.INCH), 66 - current.getX(DistanceUnit.INCH)));

            headingError = (currentHeading - targetHeading + 180) % 360 - 180; // normalize heading to -180 ~ +180

            // calculate PID for heading control
            long currentTime = System.nanoTime();
            double deltaTime = (headingLastTime == 0) ? 0.02 : (currentTime - headingLastTime) / 1e9; // convert to seconds

            headingLastTime = currentTime;
            double proportional = headingKp * headingError;
            double derivative = headingKd * ((headingError - headingLastError) / deltaTime);
            headingLastError = headingError;

            turn += (proportional + derivative) ;
        }



        realTheta = (360 - pinPoint.getPosition().getHeading(AngleUnit.DEGREES) + 90) + theta;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    public void driveConstantOriented(Gamepad gamepad, boolean chasisAutoAim) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 0.85;
        y = Math.abs(y)<=0.1 ? 0 : y;
        x = Math.abs(x)<=0.1 ? 0 : x;
        rx = Math.abs(rx)<=0.1 ? 0 : rx;
        pinPoint.update();

        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);
        turn = rx; // Fix without testing: autoaim now stopped blocking rightstick

        if (chasisAutoAim)
        {
            Pose2D current = pinPoint.getPosition();
            double currentHeading, targetHeading, headingError;
            currentHeading = current.getHeading(AngleUnit.DEGREES);
            targetHeading = Math.toDegrees(Math.atan2(teleOpTargetY - current.getY(DistanceUnit.INCH), teleOpTargetX - current.getX(DistanceUnit.INCH)));

            headingError = (currentHeading - targetHeading + 180) % 360 - 180; // normalize heading to -180 ~ +180

            // calculate PID for heading control
            long currentTime = System.nanoTime();
            double deltaTime = (headingLastTime == 0) ? 0.02 : (currentTime - headingLastTime) / 1e9; // convert to seconds

            headingLastTime = currentTime;
            double proportional = headingKp * headingError;
            double derivative = headingKd * ((headingError - headingLastError) / deltaTime);
            headingLastError = headingError;

            turn += (proportional + derivative) ;
        }



        realTheta = fixedFieldHeading + theta;
        realTheta = realTheta % 360;
        if (realTheta < 0) realTheta += 360;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        if(gamepad.right_bumper){
            leftFront.setPower(leftFrontPower*0.3);
            rightFront.setPower(rightFrontPower*0.3);
            leftBack.setPower(leftBackPower*0.3);
            rightBack.setPower(rightBackPower*0.3);
        }else{
            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower );
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower );
        }

    }

    public void driveTrainConstantOriented(Gamepad gamepad, boolean chasisAutoAim) {
        double y = -gamepad.left_stick_y, x = gamepad.left_stick_x, rx = gamepad.right_stick_x * 0.85;
        y = Math.abs(y)<=0.1 ? 0 : y;
        x = Math.abs(x)<=0.1 ? 0 : x;
        rx = Math.abs(rx)<=0.1 ? 0 : rx;
        pinPoint.update();

        theta = Math.atan2(y, x) * 180 / Math.PI;
        power = Math.hypot(x, y);
        turn = rx; // Fix without testing: autoaim now stopped blocking rightstick

        if (chasisAutoAim)
        {
            Pose2D current = pinPoint.getPosition();
            double currentHeading, targetHeading, headingError;
            currentHeading = current.getHeading(AngleUnit.DEGREES);
            targetHeading = Math.toDegrees(Math.atan2(66 - current.getY(DistanceUnit.INCH), 66 - current.getX(DistanceUnit.INCH)));

            headingError = (currentHeading - targetHeading + 180) % 360 - 180; // normalize heading to -180 ~ +180

            // calculate PID for heading control
            long currentTime = System.nanoTime();
            double deltaTime = (headingLastTime == 0) ? 0.02 : (currentTime - headingLastTime) / 1e9; // convert to seconds

            headingLastTime = currentTime;
            double proportional = headingKp * headingError;
            double derivative = headingKd * ((headingError - headingLastError) / deltaTime);
            headingLastError = headingError;

            turn += (proportional + derivative) ;
        }



        realTheta = fixedFieldHeading + theta;
        realTheta = realTheta % 360;
        if (realTheta < 0) realTheta += 360;

        double sin = Math.sin((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double cos = Math.cos((realTheta * (Math.PI / 180)) - (Math.PI / 4));
        double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFrontPower = (power * cos / maxSinCos + turn);
        double rightFrontPower = (power * sin / maxSinCos - turn);
        double leftBackPower = (power * sin / maxSinCos + turn);
        double rightBackPower = (power * cos / maxSinCos - turn);

        leftFront.setPower(leftFrontPower );
        rightFront.setPower(rightFrontPower );
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower );
    }
    public Pose2D getPosition() {
        pinPoint.update();
        return pinPoint.getPosition();
    }
}
