package frc.robot;

public class Constants {
    
    //Controller Ports
    public static final int PRIMARY_DRIVER_PORT = 0;
    public static final int SECONDARY_DRIVER_PORT = 1;
    public static final int BUTTON_BOX_PORT = 2;
    
    /* DRIVEBASE CONSTANTS */

    //CAN Ports
    public static final int LEFT_MASTER_PORT = 2;
    public static final int RIGHT_MASTER_PORT = 0;
    public static final int LEFT_SLAVE_PORT = 3;
    public static final int RIGHT_SLAVE_PORT = 1;

    //2-Speed Transmission Solenoid Ports
    public static final int DRIVE_SOLENOID_FORWARD = 0;
    public static final int DRIVE_SOLENOID_REVERSE = 1;

    //PID Constants
    public static final double[] LINEAR_GAINS = {0.33, 0.0, 10};
    public static final double[] ROTATE_GAINS = {0.024, 0.0001, 0.2};
    public static final double TURNING_CONSTANT = 0.9;
    
    //Drive Thresholds
    public static final double ROTATE_EPSILON = 3;
    public static final int LINEAR_EPSILON = 200;
    public static final int TILT_EPSILON = 15;

    //Kinematics
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double ENCODER_TICKS_PER_ROT = 4096.0 / 3.0;

    public static final double CARGO_SEARCH_ROTATE_SPEED = 0.5;
    public static final double CARGO_SEARCH_EPSILON = 3;

    public static final double DECELERATION_CONSTANT = 0.8;

    /* INTAKE ARM CONSTANTS */

    //CAN Motor Ports
    public static final int LEFT_INTAKE_PORT = 5;
    public static final int RIGHT_INTAKE_PORT = 6;
    public static final int WRIST_MOTOR_PORT = 7;

    //Intake Wheel Speed
    public static final double INTAKE_SPEED = 1.0;

    //Intake Pneumatic Ports
    public static final int INTAKE_PCM_PORT = 0;
    public static final int ARM_EXPAND_SOLENOID_FORWARD = 5;
    public static final int ARM_EXPAND_SOLENOID_REVERSE = 4;
    public static final int HATCH_SHOOTER_SOLENOID_FORWARD = 6;
    public static final int HATCH_SHOOTER_SOLENOID_REVERSE = 7;
    public static final int HATCH_GRABBER_SOLENOID_FORWARD = 2;
    public static final int HATCH_GRABBER_SOLENOID_REVERSE = 3;

    //Intake Arm Wrist Threshold
    public static final double INTAKE_WRIST_EPSILON = 0.35;

    //Intake Wrist Smart Motion Coefficients (PIDIzF)
    public static final double[] WRIST_PID = {5e-5, 1e-6, 0, 0, 0.000156};
    public static final double WRIST_MAX_RPM = 5700;
    public static final double WRIST_MAX_VEL = 2000;
    public static final double WRIST_MAX_ACC = 1500;
    public static final int WRIST_SMART_MOTION_SLOT = 0;

    /* ELEVATOR CONSTANTS */

     //Elevator Motor CAN Port
    public static final int ELEVATOR_TALON_PORT = 4;

    public static final int ELEVATOR_LIDAR_PORT = 6;

    //public static final double[] HATCH_ELEVATOR_GAINS = {0.03, 0.0, 0.05};
    //public static final double[] CARGO_ELEVATOR_GAINS = {0.03, 0.0, 0.05};  
    //TODO: Retune PID Values, new Transmission  
    public static final double[] ELEVATOR_GAINS = {7.0, 0.0, 0.0};
    public static final double ELEVATOR_EPSILON = 4.0;

    //Azimuth values for each scoring position on the field
    public static final int RIGHT_ROCKET_TARGET_AZIMUTH = 29;
    public static final int MID_ROCKET_TARGET_AZIMUTH = 90;
    public static final int LEFT_ROCKET_TARGET_AZIMUTH = 151;
    public static final int LOADING_STATION_TARGET_AZIMUTH = 180;
    public static final int MID_CARGO_TARGET_AZIMUTH = 90;
    public static final int LEFT_CARGO_TARGET_AZIMUTH = 90;
    public static final int RIGHT_CARGO_TARGET_AZIMUTH = -90;

    //Button Box button to toggle Intake Arm Tilt
    public static final int INTAKE_MODE_TOGGLE_BUTTON = 15;

    /* PISTON CLIMBER Pneumatic Ports */
    public static final int CLIMBER_PCM_PORT = 1;
    public static final int CLIMBER_FRONT_PISTON_FORWARD_PORT = 1;
    public static final int CLIMBER_FRONT_PISTON_REVERSE_PORT = 2;
    public static final int CLIMBER_REAR_PISTON_FORWARD_PORT = 0;
    public static final int CLIMBER_REAR_PISTON_REVERSE_PORT = 3;
}