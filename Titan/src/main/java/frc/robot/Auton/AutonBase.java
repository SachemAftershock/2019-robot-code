package frc.robot.Auton;

public class AutonBase {
    private AutonTask task;
    private double setpoint;

    AutonBase(AutonTask task, double setpoint) {
        this.task = task;
        this.setpoint = setpoint;
    }

    public AutonTask getTask() {
        return task;
    }

    public double getSetpoint() {
        return setpoint;
    }
}