package frc.robot.Enums;
import frc.robot.Enums.AutoObjective;

public enum IntakePosition {
    HATCH(1), CARGO(11), TOP_ROCKET_TILT(7.0);

    private final double targetEncValue;

    private IntakePosition(double targetEncValue) {
        this.targetEncValue = targetEncValue;
    }
    public double getTargetEncValue() {
        return targetEncValue;
    }
    public AutoObjective getAutoObjective(IntakePosition position) {
        if(position == HATCH) return AutoObjective.TILTHATCH;
        if(position == CARGO) return AutoObjective.TILTCARGO;
        if(position == TOP_ROCKET_TILT) return AutoObjective.TILTTOPROCKET;
        return null;
    }
}