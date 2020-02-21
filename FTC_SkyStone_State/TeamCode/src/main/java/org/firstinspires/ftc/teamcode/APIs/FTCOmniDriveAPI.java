package org.firstinspires.ftc.teamcode.APIs;

import java.lang.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.APIs.*;

public class FTCOmniDriveAPI {
    //Outputs to wheel
    double leftMotorSpeed;
    double rightMotorSpeed;
    double strafeMotorSpeed;
    double startFPosition;
    double startSPosition;
    double targetFPosition;
    double targetSPosition;

    //Motors
    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    DcMotor strafeDriveMotor;
    Telemetry telemetry;
    GyroscopeAPI gyro;
    PidAPI leftStraightPID;
    PidAPI rightStraightPID;
    PidAPI strafeStraightPID;
    PidAPI leftRotatePID;
    PidAPI rightRotatePID;
    PidAPI strafeRotatePID;

    public FTCOmniDriveAPI(HardwareMap hardwareMap) {
        this.leftDriveMotor = hardwareMap.get(DcMotor.class, "leftDrive");
        this.rightDriveMotor = hardwareMap.get(DcMotor.class, "rightDrive");
        this.strafeDriveMotor = hardwareMap.get(DcMotor.class, "strafeDrive");
    }

    public void calculateWheelSpeeds(float joystick1x, float joystick1y, float joystick2x) {
        //create 3D inputs based on joystick coordinates
        double forwardInput = (double) joystick1y;
        double strafeInput = (double) joystick1x;
        double rotationInput = -(double) joystick2x;

        double targetLeftMotorSpeed;
        double targetRightMotorSpeed;
        double targetStrafeMotorSpeed;
        double goldenRatio;

        targetLeftMotorSpeed = rotationInput+forwardInput;
        targetRightMotorSpeed = -rotationInput+forwardInput;
        targetStrafeMotorSpeed = strafeInput;

        if(Math.abs(targetLeftMotorSpeed) > 1) {
            goldenRatio = targetLeftMotorSpeed/targetRightMotorSpeed;
            targetRightMotorSpeed = targetRightMotorSpeed/goldenRatio;
            if(targetLeftMotorSpeed > 1) {
                targetLeftMotorSpeed = 1;
            } else if(targetLeftMotorSpeed < -1) {
                targetLeftMotorSpeed = -1;
            }
        } else if(Math.abs(targetRightMotorSpeed) > 1) {
            goldenRatio = targetRightMotorSpeed/targetLeftMotorSpeed;
            targetLeftMotorSpeed = targetLeftMotorSpeed/goldenRatio;
            if(targetRightMotorSpeed > 1){
                targetRightMotorSpeed = 1;
            } else if(targetRightMotorSpeed < -1){
                targetRightMotorSpeed = -1;
            }
        }

        leftMotorSpeed = targetLeftMotorSpeed;
        rightMotorSpeed = targetRightMotorSpeed;
        strafeMotorSpeed = targetStrafeMotorSpeed;
    }

    //Cnotrol the chassis as an omni drive using joystick inputs
    public void driveOmniJoystick(float leftJoystickX, float leftJoystickY, float rightJoystickX) {
        calculateWheelSpeeds(leftJoystickX, -rightJoystickX, -leftJoystickY);
        this.rightDriveMotor.setPower(rightMotorSpeed);
        this.leftDriveMotor.setPower(leftMotorSpeed);
        this.strafeDriveMotor.setPower(strafeMotorSpeed);
    }

    public void setLeftDriveMotorSpeed(double speed) {
        leftDriveMotor.setPower(speed);
    }

    public void setRightDriveMotorSpeed(double speed) {
        rightDriveMotor.setPower(speed);
    }

    public void setStrafeDriveMotorSpeed(double speed) {
        strafeDriveMotor.setPower(speed);
    }

    public void controlChassis(double leftSpeed, double rightSpeed, double strafeSpeed) {
        leftDriveMotor.setPower(leftSpeed);
        rightDriveMotor.setPower(rightSpeed);
        strafeDriveMotor.setPower(strafeSpeed);
    }
}