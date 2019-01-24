package frc.robot;

public class Constants {
    static final int PRIMARY_DRIVER_PORT = 0;
    static final int SECONDARY_DRIVER_PORT = 1;

    static final int LEFT_MASTER_PORT = 2;
    static final int RIGHT_MASTER_PORT = 3;
    static final int LEFT_SLAVE_PORT = 0;
    static final int RIGHT_SLAVE_PORT = 1;

    static final int DRIVE_SOLENOID_FORWARD = 0;
    static final int DRIVE_SOLENOID_REVERSE = 1;

    static final double[] LINEAR_GAINS = {0.25, 0.0, 3.0};
    static final double[] ROTATE_GAINS = {0.0105, 0.0, 0.03};
    static final double ROTATE_EPSILON = 3;
    static final double LINEAR_EPSILON = 100;

    static final double WHEEL_DIAMETER_INCHES = 6;
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double ENCODER_TICKS_PER_ROT = 4096.0 / 3.0;
}