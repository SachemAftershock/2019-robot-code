package frc.robot.Enums;

import frc.robot.Elevator;

public enum ElevatorPosition {
    FLOOR(0,0,0,0), LOW(0,0,0,0), MID(0,0,0,0), HIGH(0,0,0,0);

    private final int targetHatchEnc;
    private final int targetHatchLidar;
    private final int targetCargoEnc;
    private final int targetCargoLidar;

    private ElevatorPosition(int hatchEnc, int hatchLidar, int cargoEnc, int cargoLidar) {
        targetHatchEnc = hatchEnc;
        targetHatchLidar = hatchLidar;
        targetCargoEnc = cargoEnc;
        targetCargoLidar = cargoLidar;
        }

    public int getTargetEncValue() {
        return Elevator.getInstance().getIntakePosition() == IntakePosition.HATCH ? targetHatchEnc : targetCargoEnc;
    }
    public int getTargetLidarValue() {
        return Elevator.getInstance().getIntakePosition() == IntakePosition.HATCH ? targetHatchLidar : targetCargoLidar;
    }
    public AutoObjective getElevatorCmd(ElevatorPosition position) {
        if(position == LOW)
            return AutoObjective.ELEVATORLOW;
        if(position == MID)
            return AutoObjective.ELEVATORMID;
        if(position == HIGH)
            return AutoObjective.ELEVATORHIGH;;
        if(position == FLOOR)
            return AutoObjective.ELEVATORFLOOR;
        return null;
    }
}