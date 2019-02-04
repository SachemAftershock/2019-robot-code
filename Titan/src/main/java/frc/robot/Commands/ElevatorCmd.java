package frc.robot.Commands;

import frc.robot.Enums.ElevatorPosition;

public class ElevatorCmd extends BaseCmd {

    public ElevatorCmd(ElevatorPosition position, double setpoint) {
        super(position.getElevatorCmd(position), position.getTargetEncValue());
    }
}