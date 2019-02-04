package frc.robot.Enums;

public enum IntakePosition {
    HATCH(0), CARGO(0), TOP_ROCKET_TILT(0);

    private int targetEncValue;

    private IntakePosition(int targetEncValue) {
        this.targetEncValue = targetEncValue;
    }
    public int getTargetEncValue() {
        return targetEncValue;
    }
}