package frc.robot;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;

public class ElevatorInfo {
    private ElevatorPosition targetPosition;
    private IntakePosition targetIntakePosition;
    private int targetAzimuth;

    public ElevatorInfo(ElevatorPosition targetPosition,IntakePosition targetIntakePosition, int targetAzimuth) {
        this.targetPosition = targetPosition;
        this.targetIntakePosition = targetIntakePosition;
        this.targetAzimuth = targetAzimuth;
    }
    public ElevatorPosition getElevatorPosition() {
        return targetPosition;
    }
    public IntakePosition getTargetIntakePosition() {
        return targetIntakePosition;
    }
    public int getTargetAzimuth() {
        return targetAzimuth;
    }
}