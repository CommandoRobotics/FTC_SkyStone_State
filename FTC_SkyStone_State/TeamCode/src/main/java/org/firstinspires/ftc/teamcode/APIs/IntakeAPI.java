package org.firstinspires.ftc.teamcode.APIs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants.ConstantValues;

public class IntakeAPI {

    DcMotor leftIntakeMotor;
    DcMotor rightIntakeMotor;
    DcMotor tiltMotor;

    double intakeInPower = 0;
    double intakeOutPower = 0;

    public IntakeAPI(HardwareMap hardwareMap, double intakeInPower, double intakeOutPower) {
        this.leftIntakeMotor = hardwareMap.get(DcMotor.class, "leftIntake");
        this.rightIntakeMotor = hardwareMap.get(DcMotor.class, "rightIntake");
        this.tiltMotor = hardwareMap.get(DcMotor.class, "tilt");

        this.leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(ConstantValues.isLeftIntakeMotorReversed) {
            this.leftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if(ConstantValues.isRightIntakeMotorReversed) {
            this.rightIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        if(ConstantValues.isTiltMotorReversed) {
            this.tiltMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setIntakeSpeed(double speed) {
        leftIntakeMotor.setPower(speed);
        rightIntakeMotor.setPower(speed);
    }

    public void setTiltSpeed(double speed) {
        tiltMotor.setPower(speed);
    }

    public void stopIntake() {
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
    }

    public void stopTilt() {
        tiltMotor.setPower(0);
    }

    public void setIntakeInSpeed(double speed) {
        intakeInPower = speed;
    }

    public void setIntakeOutSpeed(double speed) {
        intakeOutPower = speed;
    }

    public void intakeIn() {
        leftIntakeMotor.setPower(intakeInPower);
        rightIntakeMotor.setPower(intakeInPower);
    }

    public void intakeOut() {
        leftIntakeMotor.setPower(intakeOutPower);
        rightIntakeMotor.setPower(intakeOutPower);
    }
}