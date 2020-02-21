package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.APIs.FTCOmniDriveAPI;
import org.firstinspires.ftc.teamcode.APIs.IntakeAPI;
import org.firstinspires.ftc.teamcode.Constants.ConstantValues;

@TeleOp(name="Non sketch teleop")

public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCOmniDriveAPI robot = new FTCOmniDriveAPI(hardwareMap);
        IntakeAPI intake = new IntakeAPI(hardwareMap, 0.5, 0.5);

        waitForStart();

        while(opModeIsActive()) {
            robot.driveOmniJoystick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                intake.intakeOut();
            } else if(gamepad1.right_bumper) {
                intake.intakeIn();
            } else {
                intake.stopIntake();
            }

            if(gamepad1.left_trigger > ConstantValues.tiltMotorDeadzone && !(gamepad1.right_trigger > ConstantValues.tiltMotorDeadzone)) {
                intake.setTiltSpeed(gamepad1.left_trigger);
            } else if(gamepad1.right_trigger > ConstantValues.tiltMotorDeadzone) {
                intake.setTiltSpeed(gamepad1.right_trigger);
            } else {
                intake.stopTilt();
            }
        }

    }

}