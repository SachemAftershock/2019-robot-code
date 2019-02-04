package frc.robot;

public class Constants {
    public static final int PRIMARY_DRIVER_PORT = 0;
    public static final int SECONDARY_DRIVER_PORT = 1;
    public static final int BUTTON_BOX_PORT = 2;

    public static final int LEFT_MASTER_PORT = 2;
    public static final int RIGHT_MASTER_PORT = 3;
    public static final int LEFT_SLAVE_PORT = 0;
    public static final int RIGHT_SLAVE_PORT = 1;

    public static final int DRIVE_SOLENOID_FORWARD = 0;
    public static final int DRIVE_SOLENOID_REVERSE = 1;

    public static final int TILT_THRESHOLD = 15;

    public static final double[] LINEAR_GAINS = {0.75, 0.0, 10};
    public static final double[] ROTATE_GAINS = {0.0105, 0.01, 0.03};
    public static final double ROTATE_EPSILON = 3;
    public static final int LINEAR_EPSILON = 200;

    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double ENCODER_TICKS_PER_ROT = 4096.0 / 3.0;

    public static final int LEFT_INTAKE_PORT = 4;
    public static final int RIGHT_INTAKE_PORT = 5;
    public static final int TILT_MOTOR_PORT = 6;

    //TODO: Finalize all Numbers Below
    public static final double INTAKE_SPEED = 0.6;
    public static final double[] TILT_GAINS = {0.0, 0.0, 0.0, 0.0, 0.0};
    public static final int HATCH_SOLENOID_FORWARD = 2;
    public static final int HATCH_SOLENOID_REVERSE = 3;
    public static final int INTAKE_MODE_TOGGLE_BUTTON = 14;
    public static final int CARGO_BUTTON_PORT = 0;

    public static final int ELEVATOR_TALON_PORT = 6;
    public static final int ELEVATOR_LIDAR_PORT = 3;
    public static final int TOP_LS_PORT = 1;
    public static final int BOTTOM_LS_PORT = 2;
    public static final double[] ELEVATOR_GAINS = {0.0, 0.0, 0.0, 0.0};
    public static final int ENC_THRESHOLD = 75;
    public static final double LIDAR_THRESHOLD = -1;

    public static final int RIGHT_ROCKET_TARGET_AZIMUTH = -1;
    public static final int MID_ROCKET_TARGET_AZIMUTH = -1;
    public static final int LEFT_ROCKET_TARGET_AZIMUTH = -1;
    public static final int LOADING_STATION_TARGET_AZIMUTH = -1;
    public static final int MID_CARGO_TARGET_AZIMUTH = -1;
    public static final int LEFT_CARGO_TARGET_AZIMUTH = -1;
    public static final int RIGHT_CARGO_TARGET_AZIMUTH = -1;
}