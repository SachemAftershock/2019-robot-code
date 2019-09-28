package frc.robot.Enums;

/**
 * Enum for Intake Arm Tilt Position
 * 
 * @author Shreyas Prasad
 */
public enum IntakePosition {
    HATCH(0), CARGO(11.75), TOP_ROCKET_TILT(7.0);

    private final double targetEncValue;

    /**
     * Contructor for IntakePosition Enum
     * 
     * @param targetEncValue Tilt Encoder Value corresponding to the Intake Position
     */
    private IntakePosition(double targetEncValue) {
        this.targetEncValue = targetEncValue;
    }

    /**
     * Gets target encoder value for the corresponding Intake Position
     * 
     * @return Encoder Value for the corresponding Intake Position
     */
    public double getTargetEncValue() {
        return targetEncValue;
    }
}