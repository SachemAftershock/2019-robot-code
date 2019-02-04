package frc.robot;
import frc.robot.Enums.ElevatorPosition;

public class ElevatorInfo {
    private ElevatorPosition targetPosition;
    private int targetAzimuth;

    public ElevatorInfo(ElevatorPosition targetPosition, int targetAzimuth) {
        this.targetPosition = targetPosition;
        this.targetAzimuth = targetAzimuth;
    }
    public ElevatorPosition getElevatorPosition() {
        return targetPosition;
    }
    public int getTargetAzimuth() {
        return targetAzimuth;
    }
}