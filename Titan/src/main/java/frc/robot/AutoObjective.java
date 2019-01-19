package frc.robot;

public enum AutoObjective {
    LINEAR, ROTATE;
}

enum Distance
{
    INCH(1.0), METER(39.37), FOOT(12.0);

    private double conversionFactor;
    Distance(double conversionFactor) {
        this.conversionFactor = conversionFactor;
    }

    public double getConversionFactor() {
        return conversionFactor / Constants.WHEEL_CIRCUMFERENCE * Constants.ENCODER_TICKS_PER_ROT;
    }
}

enum Rotation
{
    CLOCKWISE(1), COUNTERCLOCKWISE(-1);

    private int direction;
    Rotation(int direction) {
        this.direction = direction;
    }

    public int getDirection() {
        return direction;
    }
}

class AutoTask {
    private AutoObjective objective;
    private double setpoint;

    public AutoTask(AutoObjective objective, double setpoint) {
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
    