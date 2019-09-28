package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * PID Controller Class
 * 
 * @author Rohan Bapat
 */
class PIDController {
    private double integral, derivative, error, prevError;
    private double[] gains;
    private Timer timer;

    /**
     * PID Constructor
     */
    public PIDController() {
        gains = new double[3];
        integral = 0.0;
        derivative = 0.0;
        error = 0.0;
        prevError = 0.0;

        timer = new Timer();
    }

    /**
     * Wrapper method to update current position and new setpoint
     * 
     * @param current current position
     * 
     * @param setpoint target position
     * 
     * @return PID Processed output value
     */
    public double update(double current, double setpoint) {
        error = setpoint - current;
        double output = this.updateError(error);
        return output;
    }

    /**
     * Wrapper method to update current and target position for rotational PID
     * 
     * @param current current position
     * 
     * @param setpoint target position
     * 
     * @return PID Processed output value
     */
    public double updateRotation(double current, double setpoint) {
        error = Utilities.rotationalError(current, setpoint);
        return this.updateError(error);
    }

    /**
     * PID Processor Method
     * 
     * @param error distance from the target position
     * 
     * @return PID Processed output value
     */
    private double updateError(double error) {
        this.error = error;
        integral += error;
        derivative = error - prevError;

        double output = gains[0] * error + gains[1] * integral + gains[2] * derivative;
        prevError = error;
        if(Math.abs(output) > 1.0) {
            output /= output < 0 ? -output : output;
        }
        return output;
    }

    /**
     * Begin PID Controller by setting Gain Values and starting timer
     * 
     * @param gains array of Gain Values in PID form
     */
    public void start(double[] gains) {
        this.gains = gains;
        integral = 0.0;
        derivative = 0.0;
        error = 0.0;
        prevError = 0.0;
        timer.start();
    }

    /**
     * Get Error Value
     * 
     * @return Error: current from the setpoint
     */
    public double getError() {
        return error;
    }

    /**
     * Get Total Time PID has been running
     * 
     * @return time PID has run for
     */
    public double getRunTime() {
        return timer.get();
    }
}