package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_TRANS;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.MID_TRANS;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_TRANS;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake;
    private CRServo leftTrans;
    private CRServo rightTrans;
    private CRServo midTrans;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        leftTrans = hardwareMap.get(CRServo.class, LEFT_TRANS);
        rightTrans = hardwareMap.get(CRServo.class, RIGHT_TRANS);
        midTrans = hardwareMap.get(CRServo.class, MID_TRANS);

        rightTrans.setDirection(DcMotorSimple.Direction.REVERSE);
        midTrans.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void intakeIn() {
        intake.setPower(1);
    }

    public void intakeOut() {
        intake.setPower(-1);
    }

    public void intakeStop() {
        intake.setPower(0);
    }

    public void leftUp() {
        leftTrans.setPower(1);
    }

    public void leftDown() {
        leftTrans.setPower(-1);
    }

    public void leftStop() {
        leftTrans.setPower(0);
    }

    public void rightUp() {
        rightTrans.setPower(1);
    }

    public void rightDown() {
        rightTrans.setPower(-1);
    }

    public void rightStop() {
        rightTrans.setPower(0);
    }

    public void midUp() {
        midTrans.setPower(1);
    }
    public void transferToShooter() {
        midUp();
        leftUp();
        rightUp();
    }
    public void stopServo() {
        midStop();
        leftStop();
        rightStop();
    }

    public void midDown() {
        midTrans.setPower(-1);
    }

    public void midStop() {
        midTrans.setPower(0);
    }

    public void servoStop() {
        leftStop();
        rightStop();
        midStop();
    }
}
