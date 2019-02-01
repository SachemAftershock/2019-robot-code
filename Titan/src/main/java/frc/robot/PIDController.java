package frc.robot;

import edu.wpi.first.wpilibj.Timer;

class PIDController {

    private double setpoint, integral, derivative, error, prevError;
    private double[] gains;
    private Timer timer;

    public PIDController() {
        gains = new double[3];
        setpoint = 0.0;
        integral = 0.0;
        derivative = 0.0;
        error = 0.0;
        prevError = 0.0;

        timer = new Timer();
    }

    public double update(double current) {
        error = setpoint - current;
        integral += error;
        derivative = error - prevError;

        double output = gains[0] * error + gains[1] * integral + gains[2] * derivative;
        prevError = error;
        return Math.abs(output) > 1.0 ? output / output : output;
    }

    public void start(double[] gains, double setpoint) {
        this.gains = gains;
        this.setpoint = setpoint;
        integral = 0.0;
        derivative = 0.0;
        error = 0.0;
        prevError = 0.0;
        timer.start();
    }

    public double getError() {
        return error;
    }

    public double getRunTime() {
        return timer.get();
    }
}