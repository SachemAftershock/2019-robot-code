package frc.robot;

class BaseCmd {
    enum AutoObjective {DRIVELINEAR, DRIVEROTATE};

    AutoObjective objective;
    double setpoint;

    public BaseCmd(AutoObjective objective, double setpoint) {
        this.objective = objective;
        this.setpoint = setpoint;
    }

    public AutoObjective getObjective() {
        return objective;
    }

    public double getSetpoint() {
        return setpoint;
    }

}