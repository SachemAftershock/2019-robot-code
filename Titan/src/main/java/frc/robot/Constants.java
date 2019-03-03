package frc.robot;

public class Constants { //TODO: Clean up
    public static final int PRIMARY_DRIVER_PORT = 0;
    public static final int SECONDARY_DRIVER_PORT = 1;
    public static final int BUTTON_BOX_PORT = 2;

    public static final int LEFT_MASTER_PORT = 2;
    public static final int RIGHT_MASTER_PORT = 0;
    public static final int LEFT_SLAVE_PORT = 3;
    public static final int RIGHT_SLAVE_PORT = 1;

    public static final int DRIVE_SOLENOID_FORWARD = 0;
    public static final int DRIVE_SOLENOID_REVERSE = 1;

    public static final double[] LINEAR_GAINS = {0.33, 0.0, 10};
    public static final double[] ROTATE_GAINS = {0.024, 0.0001, 0.2};
    public static final double TURNING_CONSTANT = 0.9;
    public static final double ROTATE_EPSILON = 3;
    public static final int LINEAR_EPSILON = 200;
    public static final int TILT_EPSILON = 15;

    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double ENCODER_TICKS_PER_ROT = 4096.0 / 3.0;

    public static final int LEFT_INTAKE_PORT = 5;
    public static final int RIGHT_INTAKE_PORT = 6;
    public static final int TILT_MOTOR_PORT = 7;

    public static final double INTAKE_SPEED = 0.6;
    public static final double[] TILT_GAINS = {0.01, 0.0, 0.0, 0.0, 0.0};
    public static final int LEFT_HATCH_SOLENOID_FORWARD = 4;
    public static final int LEFT_HATCH_SOLENOID_REVERSE = 5;
    public static final int RIGHT_HATCH_SOLENOID_FORWARD = 6;
    public static final int RIGHT_HATCH_SOLENOID_REVERSE = 7;
    public static final int INTAKE_MODE_TOGGLE_BUTTON = 15;
    public static final int CARGO_BUTTON_PORT = 3;
    public static final double TILT_THRESHOLD = 0.35;

    public static final int ELEVATOR_TALON_PORT = 4;
    public static final int ELEVATOR_LIDAR_PORT = 6;
    public static final int TOP_LS_PORT = 4;
    public static final int BOTTOM_LS_PORT = 5;
    public static final double[] HATCH_ELEVATOR_GAINS = {0.03, 0.0, 0.05};
    public static final double[] CARGO_ELEVATOR_GAINS = {0.03, 0.0, 0.05};
    public static final int ELEVATOR_ENC_THRESHOLD = 75;
    public static final double LIDAR_THRESHOLD = 4.0;

    public static final int RIGHT_ROCKET_TARGET_AZIMUTH = 29;
    public static final int MID_ROCKET_TARGET_AZIMUTH = 90;
    public static final int LEFT_ROCKET_TARGET_AZIMUTH = 151;
    public static final int LOADING_STATION_TARGET_AZIMUTH = 180;
    public static final int MID_CARGO_TARGET_AZIMUTH = 90;
    public static final int LEFT_CARGO_TARGET_AZIMUTH = 90;
    public static final int RIGHT_CARGO_TARGET_AZIMUTH = -90;

    /*
    public static final int CLIMBER_BACK_LEFT_SPARK_PORT = -1;
    public static final int CLIMBER_BACK_RIGHT_SPARK_PORT = -1;
    public static final int CLIMBER_FRONT_LEFT_SPARK_PORT = -1;
    public static final int CLIMBER_FRONT_RIGHT_SPARK_PORT = -1;
    public static final double EXTEND_SPEED = 0.3;
    public static final int CLIMBER_TOP_BACK_LS = 3;
    public static final int CLIMBER_TOP_FRONT_LS = 4;
    public static final int CLIMBER_BOTTOM_BACK_LS = 5;
    public static final int CLIMBER_BOTTOM_FRONT_LS = 6;
    public static final int CLIMB_BOTTOM_WHEELS_TALON = 11;
    public static final double CLIMBER_DRIVE_SPEED = 0.2;
    public static final double HAB_DRIVE_SPEED = 0.3;
    public static final float PITCH_LIMIT = 35;
    public static final float PITCH_TO_SPEED_MODIFIER = (float) 0.333333;
    public static final double HAB_DRIVE_OFF_DISTANCE = -1.0;
    */
    public static final int CLIMBER_PCM_PORT = 1;
    public static final int CLIMBER_FRONT_PISTON_FORWARD_PORT = 0;
    public static final int CLIMBER_FRONT_PISTON_REVERSE_PORT = 3;
    public static final int CLIMBER_REAR_PISTON_FORWARD_PORT = 2;
    public static final int CLIMBER_REAR_PISTON_REVERSE_PORT = 1;

    public static final double CARGO_SEARCH_ROTATE_SPEED = 0.5;
    public static final double CARGO_SEARCH_EPSILON = 3;

    public static final double DECELERATION_CONSTANT = 0.8;
}