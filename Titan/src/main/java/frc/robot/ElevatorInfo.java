package frc.robot;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

/**
 * Informational Class in ELevator Map to map Button Box Button ID Values to:
 * an Elevator Position, an Intake Position, and a target Azimuth value
 * 
 * @author Shreyas Prasad
 */
public class ElevatorInfo {
    private ElevatorPosition targetPosition;
    private IntakePosition targetIntakePosition;
    private int targetAzimuth;

    /**
     * Elevator Info Class Constructor
     * 
     * @param targetPosition target Elevator Position
     * 
     * @param targetIntakePosition target Intake Position needed at the Elevator Position
     * 
     * @param targetAzimuth target Azimuth the robot must be at in order to score; unused
     */
    public ElevatorInfo(ElevatorPosition targetPosition,IntakePosition targetIntakePosition, int targetAzimuth) {
        this.targetPosition = targetPosition;
        this.targetIntakePosition = targetIntakePosition;
        this.targetAzimuth = targetAzimuth;
    }

    /**
     * Get target ELevator Position
     * 
     * @return target Elevator Position
     */
    public ElevatorPosition getElevatorPosition() {
        return targetPosition;
    }

    /**
     * Get target Intake Position
     * 
     * @return target Intake Position
     */
    public IntakePosition getTargetIntakePosition() {
        return targetIntakePosition;
    }

    /**
     * Get target Azimuth for Robot; unused
     * 
     * @return target Azimuth value for robot
     */
    public int getTargetAzimuth() {
        return targetAzimuth;
    }
}