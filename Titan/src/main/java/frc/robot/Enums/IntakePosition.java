package frc.robot.Enums;
import frc.robot.Enums.AutoObjective;

public enum IntakePosition {
    HATCH(0), CARGO(10), TOP_ROCKET_TILT(3);

    private final int targetEncValue;

    private IntakePosition(int targetEncValue) {
        this.targetEncValue = targetEncValue;
    }
    public int getTargetEncValue() {
        return targetEncValue;
    }
    public AutoObjective getAutoObjective(IntakePosition position) {
        if(position == HATCH) return AutoObjective.TILTHATCH;
        if(position == CARGO) return AutoObjective.TILTCARGO;
        if(position == TOP_ROCKET_TILT) return AutoObjective.TILTTOPROCKET;
        return null;
    }
}