package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_FRONT;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.PANEL;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_BACK;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_FRONT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
public class ServoPositionTest extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();

    private double x, y, rx, p;
    private String[] names = {PANEL};
    private int index = 0, servoNum = names.length;
    private Servo[] servos = new Servo[servoNum];
    private double[] poses = new double[servoNum];
    private boolean upHold = false, downHold = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        for (int i = 0; i < servoNum; i++) {
            servos[i] = hardwareMap.get(Servo.class, names[i]);
            poses[i] = 0.5;
        }

        waitForStart();
        while (opModeIsActive()) {
           drivetrain.drive(gamepad1, 1, true);

//            leftFront.setPower(gamepad1.x ? 1 : 0);
//            leftBack.setPower(gamepad1.a ? 1 : 0);
//            rightFront.setPower(gamepad1.y ? 1 : 0);
//            rightBack.setPower(gamepad1.b ? 1 : 0);

            if (gamepad1.dpad_up && !upHold && index < servoNum - 1) {
                upHold = true;
                index++;
            } else if (!gamepad1.dpad_up) {
                upHold = false;
            }
            if (gamepad1.dpad_down && !downHold && index > 0) {
                downHold = true;
                index--;
            } else if (!gamepad1.dpad_down) {
                downHold = false;
            }

            poses[index] = Math.min(1.0, Math.max(0.0, (gamepad1.right_trigger - gamepad1.left_trigger) / 1300 + poses[index]));

            if (gamepad1.right_bumper)
                servos[index].setPosition(poses[index]);

            telemetry.addData("Current Servo", String.valueOf(names[index]));
            for (int i = 0; i < servoNum; i++) {
                telemetry.addData(i + " " + names[i], String.valueOf(poses[i]));
            }
            telemetry.update();
        }
    }
}
