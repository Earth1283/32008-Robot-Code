package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.robotConfigs.INTAKE;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_TRANS_SERVO1;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.LEFT_TRANS_SERVO2;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_TRANS_SERVO1;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.RIGHT_TRANS_SERVO2;
import static org.firstinspires.ftc.teamcode.constants.robotConfigs.TRANSMOTOR;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intake;

    private DcMotorEx transMotor;
    private CRServo leftTrans1;
    private CRServo rightTrans1;

    private CRServo leftTrans2;
    private CRServo rightTrans2;


    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, INTAKE);
        leftTrans1 = hardwareMap.get(CRServo.class, LEFT_TRANS_SERVO1);
        rightTrans1 = hardwareMap.get(CRServo.class, RIGHT_TRANS_SERVO1);
        leftTrans2 = hardwareMap.get(CRServo.class, LEFT_TRANS_SERVO2);
        rightTrans2 = hardwareMap.get(CRServo.class, RIGHT_TRANS_SERVO2);
        transMotor = hardwareMap.get(DcMotorEx.class, TRANSMOTOR);

        leftTrans1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTrans2.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTrans1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftTrans2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transMotor.setDirection(DcMotorEx.Direction.REVERSE);

    }

    public void intakeOut() {
        intake.setPower(1);
        rightTransOut();
        leftTransOut();
    }
    public void intakeIn() {
        intake.setPower(-1);
        rightTransIn();
        leftTransIn();
    }
    public void intakeStop() {
        intake.setPower(0);
        rightTransStop();
        leftTransStop();
    }
    public void rightTransIn(){
        rightTrans1.setPower(-1);
        rightTrans2.setPower(-1);
    }
    public void rightTransOut(){
        rightTrans1.setPower(1);
        rightTrans2.setPower(1);
    }
    public void rightTransStop(){
        rightTrans1.setPower(0);
        rightTrans2.setPower(0);
    }

    public void leftTransIn(){
        leftTrans1.setPower(1);
        leftTrans2.setPower(1);
    }
    public void leftTransOut(){
        leftTrans1.setPower(-1);
        leftTrans2.setPower(-1);
    }
    public void autoTransferToShooter() {
        transMotor.setPower(1);
        rightTransIn();
        leftTransIn();
    }
    public void servoTransIn(){
        leftTrans1.setPower(1);
        leftTrans2.setPower(1);
        rightTrans1.setPower(1);
        rightTrans2.setPower(1);
    }
    public void transferToShooterUPFirst() {
        transMotor.setPower(1);
    }
    public void transferToShooter() {
        transMotor.setPower(1);
    }
    public void leftTransStop(){
        leftTrans1.setPower(0);
        leftTrans2.setPower(0);
    }
//    public void transferToShooter() {
//        transMotor.setPower(1);
//    }
    public void transferToShooterUP() {
        transMotor.setPower(1);
    }
    public void transferToShooterStop() {
        transMotor.setPower(0);
    }
    public void transferToShooterDown() {
        transMotor.setPower(-1);
    }

    public void servoStop() {
        leftTransStop();
        rightTransStop();
    }

}
