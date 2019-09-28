package frc.robot.Commands;

import frc.robot.Enums.AutoObjective;
import frc.robot.Enums.ElevatorPosition;

public class ElevatorCmd extends BaseCmd { 
    public ElevatorCmd(ElevatorPosition position) {
        super(AutoObjective.MOVE_ELEVATOR, position.getTargetLidarValue());
    }
}