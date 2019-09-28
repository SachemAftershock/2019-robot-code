package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {
    private static Elevator instance = new Elevator();
    private TalonSRX elevatorTalon;
    private LIDAR lidar;
    private PIDController pid;

    private Elevator() { 
        super();
        elevatorTalon = new TalonSRX(Constants.ELEVATOR_TALON_PORT);
        lidar = new LIDAR(new DigitalInput(Constants.ELEVATOR_LIDAR_PORT));
        pid = new PIDController();

        elevatorTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void moveElevator(double targetValue) {
        if(Superstructure.getInstance().getSetpointReached()) {
            pid.start(Constants.ELEVATOR_GAINS);
            Superstructure.getInstance().setSetpointReached(false);
        }
        //TODO: Find out if this works okay, the elevator motor is geared hella high so this might work
        double output = 1.0;//pid.update(lidar.getDistanceCm(), targetValue);
        Superstructure.getInstance().setSetpointReached(Math.abs(pid.getError()) < Constants.ELEVATOR_EPSILON);
        elevatorTalon.set(ControlMode.PercentOutput, output);
        if(Superstructure.getInstance().getSetpointReached()) {
            elevatorTalon.set(ControlMode.PercentOutput, 0.0);
        }

        /*
        if(!Utilities.withinEpsilon(targetValue, lidar.getDistanceCm(), Constants.ELEVATOR_EPSILON)) {

        } else {
            Superstructure.getInstance().setSetpointReached(true);
        }
        */
    }

    public void driveElevator(double speed) {
        elevatorTalon.set(ControlMode.PercentOutput, speed);
    }

    public static Elevator getInstance() {
        return instance;
    }
}