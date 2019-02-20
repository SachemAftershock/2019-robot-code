package frc.robot;

import edu.wpi.first.wpilibj.Timer;

class PIDController {

    private double integral, derivative, error, prevError;
    private double[] gains;
    private Timer timer;

    public PIDController() {
        gains = new double[3];
        integral = 0.0;
        derivative = 0.0;
        error = 0.0;
        prevError = 0.0;

        timer = new Timer();
    }

    public double update(double current, double setpoint) {
        error = setpoint - current;
        double output = this.updateError(error);
        return output;
    }

    public double updateRotation(double current, double setpoint) {
        error = Utilities.rotationalError(current, setpoint);
        return this.updateError(error);
    }

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

    public void start(double[] gains) {
        this.gains = gains;
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