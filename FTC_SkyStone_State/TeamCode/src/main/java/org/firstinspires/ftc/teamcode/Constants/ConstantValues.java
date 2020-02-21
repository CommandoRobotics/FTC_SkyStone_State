package org.firstinspires.ftc.teamcode.Constants;

public class ConstantValues {

    // Motor directions
    public static final boolean isLeftIntakeMotorReversed = false;
    public static final boolean isRightIntakeMotorReversed = false;
    public static final boolean isTiltMotorReversed = false;
    public static final boolean isLeftDriveMotorReversed = false;
    public static final boolean isRightDriveMotorReversed = false;
    public static final boolean isStrafeDriveMotorReverse = false;

    // Gear ratios
    public static final double tiltGearRatio = 3; // The tilt motor gear ratio as x:1
    public static final double chassisGearRatio = 1; // The chassis gear ratio as x:1
    public static final double strafeGearRatio = 1; // The strafe wheel gear ratio as x:1

    // Pulses per revolution
    public static final int pulsePerRevolution = 1120;

    // Encoder stuff
    public static final int chassisWheelDiameterInInches = 4;
    public static final double chassWheelRadiusInInches = chassisWheelDiameterInInches/2;
    public static final double chassisWheelCircumferenceInInches = 2*chassisWheelDiameterInInches*3.14159;
    public static final double chassisDistancePerPulse = chassisGearRatio*(chassisWheelCircumferenceInInches/pulsePerRevolution);


    // Color sensors
    public static final int isBlackRedMin = 140;
    public static final int isBlackRedMax = 155;
    public static final int isBlackGreenMin = 285;
    public static final int isBlackGreenMax = 260;

    public static final int isYellowRedMin = 165;
    public static final int isYellowRedMax = 250;
    public static final int isYellowGreenMin = 280;
    public static final int isYellowGreenMax = 410;

    // Misc
    public static final double tiltMotorDeadzone = 0.05;
}