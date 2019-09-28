package frc.robot.Enums;

import frc.robot.Intake;

/**
 * Enum for Elevator Position
 * 
 * @author Shreyas Prasad
 */
public enum ElevatorPosition {
    FLOOR(37,37), LOW(16,84), MID(86,132), HIGH(160,183);

    private final int targetHatchLidar;
    private final int targetCargoLidar;

    /**
     * Constructor for ElevatorPosition Enum
     * 
     * @param hatchLidar Lidar value for Hatch Position
     * 
     * @param cargoLidar Lidar Value for Cargo Position
     */
    private ElevatorPosition(int targetHatchLidar, int targetCargoLidar) {
        this.targetHatchLidar = targetHatchLidar;
        this.targetCargoLidar = targetCargoLidar;
    }

    /**
     * Gets Target Lidar Position based on Intake Position
     * 
     * @return Target LIDAR position for corresponding Intake Position
     */
    public int getTargetLidarValue() {
        return targetHatchLidar;//Intake.getInstance().getIntakePosition() == IntakePosition.HATCH ? targetHatchLidar : targetCargoLidar;
    }
}