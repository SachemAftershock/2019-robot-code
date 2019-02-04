package frc.robot;

public class Constants {
    public static final int PRIMARY_DRIVER_PORT = 0;
    public static final int SECONDARY_DRIVER_PORT = 1;

    public static final int LEFT_MASTER_PORT = 2;
    public static final int RIGHT_MASTER_PORT = 3;
    public static final int LEFT_SLAVE_PORT = 0;
    public static final int RIGHT_SLAVE_PORT = 1;

    public static final int DRIVE_SOLENOID_FORWARD = 0;
    public static final int DRIVE_SOLENOID_REVERSE = 1;

    public static final double[] LINEAR_GAINS = {0.75, 0.0, 10};
    public static final double[] ROTATE_GAINS = {0.016, 0.0001, 0.18};
    public static final double ROTATE_EPSILON = 3;
    public static final int LINEAR_EPSILON = 200;
    public static final int TILT_EPSILON = 15;

    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double ENCODER_TICKS_PER_ROT = 4096.0 / 3.0;
}