package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_SHOOTER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp(name = "PID Tuner", group = "Tuner")
public class PIDTuner extends LinearOpMode {
    // 电机声明
    private DcMotorEx right_motor;
    private DcMotorEx left_motor;

    // PID参数变量
    private double Kp = 30.0;  // 比例系数
    private double Ki = 2.0; // 积分系数
    private double Kd = 5.0; // 微分系数
    private double Kf = 10.0;   // 前馈系数

    // 调整步长
    private final double KP_STEP = 1;
    private final double KI_STEP = 1;
    private final double KD_STEP = 1;
    private final double KF_STEP = 1;

    // 目标速度
    private double rightTargetVelocity = 300; // RPM
    private double leftTargetVelocity = 300; // RPM


    @Override
    public void runOpMode() {
        // 初始化电机
        right_motor = hardwareMap.get(DcMotorEx.class, RIGHT_SHOOTER);
        left_motor = hardwareMap.get(DcMotorEx.class, LEFT_SHOOTER);

        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_motor.setDirection(DcMotorEx.Direction.FORWARD);
        right_motor.setDirection(DcMotorEx.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // 等待开始
        telemetry.addData("状态", "准备就绪，等待开始...");
        telemetry.update();
        waitForStart();

        // 主循环
        while (opModeIsActive()) {
            // 处理手柄输入 - 调整PID参数
            handleGamepadInput();

            // 应用新的PID参数到电机
            applyRightPIDCoefficients();

            // 控制电机速度
            right_motor.setVelocity(rightTargetVelocity);
            left_motor.setVelocity(leftTargetVelocity);

            // 显示调试信息
            displayTelemetry();
        }
    }

    /**
     * 处理手柄输入，调整PID参数
     */
    private void handleGamepadInput() {
        // 调整目标速度
        if (gamepad1.dpad_up) {
            rightTargetVelocity += 50;
            leftTargetVelocity += 50;
        }
        if (gamepad1.dpad_down) {
            rightTargetVelocity -= 50;
            leftTargetVelocity -= 50;
        }
        rightTargetVelocity = Math.max(0, Math.min(rightTargetVelocity, 3000)); // 限制范围


        if (gamepad1.y) {
            rightTargetVelocity = 1800;
            leftTargetVelocity = 1800;
        }
        if (gamepad1.a) {
            rightTargetVelocity = 1600;
            leftTargetVelocity = 1600;
        }
        if (gamepad1.x) {
            rightTargetVelocity = 1500;
            leftTargetVelocity = 1500;
        }
        if (gamepad1.b) {
            rightTargetVelocity = 300;
            leftTargetVelocity = 300;
        }
        // 调整P参数 - 左摇杆上下
        if (gamepad1.left_stick_y > 0.5) Kp += KP_STEP;  // 摇杆向上，增加P
        if (gamepad1.left_stick_y < -0.5) Kp -= KP_STEP; // 摇杆向下，减小P
        Kp = Math.max(0, Kp); // 确保不为负

        // 调整I参数 - 左摇杆左右
        if (gamepad1.left_stick_x > 0.5) Ki += KI_STEP;  // 摇杆向右，增加I
        if (gamepad1.left_stick_x < -0.5) Ki -= KI_STEP; // 摇杆向左，减小I
        Ki = Math.max(0, Ki);

        // 调整D参数 - 右摇杆上下
        if (gamepad1.right_stick_y > 0.5) Kd += KD_STEP;  // 摇杆向上，增加D
        if (gamepad1.right_stick_y < -0.5) Kd -= KD_STEP; // 摇杆向下，减小D
        Kd = Math.max(0, Kd);

        // 调整F参数 - 右摇杆左右
        if (gamepad1.right_stick_x > 0.5) Kf += KF_STEP;  // 摇杆向右，增加F
        if (gamepad1.right_stick_x < -0.5) Kf -= KF_STEP; // 摇杆向左，减小F
        Kf = Math.max(0, Kf);


        // 调整P参数 - 左摇杆上下
        if (gamepad2.left_stick_y > 0.5) Kp += KP_STEP;  // 摇杆向上，增加P
        if (gamepad2.left_stick_y < -0.5) Kp -= KP_STEP; // 摇杆向下，减小P
        Kp = Math.max(0, Kp); // 确保不为负

        // 调整I参数 - 左摇杆左右
        if (gamepad2.left_stick_x > 0.5) Ki += KI_STEP;  // 摇杆向右，增加I
        if (gamepad2.left_stick_x < -0.5) Ki -= KI_STEP; // 摇杆向左，减小I
        Ki = Math.max(0, Ki);

        // 调整D参数 - 右摇杆上下
        if (gamepad2.right_stick_y > 0.5) Kd += KD_STEP;  // 摇杆向上，增加D
        if (gamepad2.right_stick_y < -0.5) Kd -= KD_STEP; // 摇杆向下，减小D
        Kd = Math.max(0, Kd);

        // 调整F参数 - 右摇杆左右
        if (gamepad2.right_stick_x > 0.5) Kf += KF_STEP;  // 摇杆向右，增加F
        if (gamepad2.right_stick_x < -0.5) Kf -= KF_STEP; // 摇杆向左，减小F
        Kf = Math.max(0, Kf);

        // 按钮快速调整步长
        if (gamepad1.a) {
            // A按钮：微调模式（小步长）
            setStepSizes(KP_STEP, KI_STEP, KD_STEP, KF_STEP);
        }
        if (gamepad1.b) {
            // B按钮：粗调模式（大步长）
            setStepSizes(KP_STEP * 5, KI_STEP * 5, KD_STEP * 5, KF_STEP * 5);
        }

        // 重置PID参数
        if (gamepad1.back) {
            resetPIDParameters();
        }

        // 防止连续快速调整
        sleep(100);
    }

    /**
     * 应用PID系数到电机
     */
    private void applyRightPIDCoefficients() {

        PIDFCoefficients coefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        right_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }

    private void applyLeftPIDCoefficients() {
        PIDFCoefficients leftcoefficients = new PIDFCoefficients(Kp, Ki, Kd, Kf);
        left_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftcoefficients);
    }

    /**
     * 设置调整步长
     */
    private void setStepSizes(double kpStep, double kiStep, double kdStep, double kfStep) {
        // 步长已在常量中定义，这里主要为了逻辑清晰
    }

    /**
     * 重置PID参数到默认值
     */
    private void resetPIDParameters() {
        Kp = 10;
        Ki = 0.0;
        Kd = 0.0;
        Kf = 0.0;
        rightTargetVelocity = 2500;
    }

    /**
     * 显示调试信息
     */
    private void displayTelemetry() {
        telemetry.addLine("=== PID调参器 ===");
        telemetry.addLine("当前PID参数:");
        telemetry.addData("目标速度", "%.0f RPM", rightTargetVelocity);
        telemetry.addData("P (比例)", "%.4f", Kp);
        telemetry.addData("I (积分)", "%.4f", Ki);
        telemetry.addData("D (微分)", "%.4f", Kd);
        telemetry.addData("F (前馈)", "%.4f", Kf);
        telemetry.addLine("");

        telemetry.addLine("右电机状态:");
        telemetry.addData("实际速度", "%.0f RPM", right_motor.getVelocity());
        telemetry.addData("速度误差", "%.0f RPM", (rightTargetVelocity - right_motor.getVelocity()));
        telemetry.addData("是否达到目标", right_motor.getVelocity() >= rightTargetVelocity * 0.95 ? "是" : "否");

        telemetry.addLine("左电机状态:");
        telemetry.addData("实际速度", "%.0f RPM", left_motor.getVelocity());
        telemetry.addData("速度误差", "%.0f RPM", (leftTargetVelocity - left_motor.getVelocity()));
        telemetry.addData("是否达到目标", left_motor.getVelocity() >= leftTargetVelocity * 0.95 ? "是" : "否");


        telemetry.update();
    }
}