package frc.robot.Commands;

import frc.robot.Enums.AutoObjective;

/**
 * Base Command Structure
 * 
 * @author Rohan Bapat
 */
public class BaseCmd {
    AutoObjective objective;
    double setpoint;

    /**
     * Command Constructor Structure
     * 
     * @param objective AutoObjective relating to the task to be performed by this command
     * 
     * @param setpoint value to be used by command
     */
    public BaseCmd(AutoObjective objective, double setpoint) {
        this.objective = objective;
        this.setpoint = setpoint;
    }

    public BaseCmd(AutoObjective objective) {
        this.objective = objective;
        this.setpoint =  -1.0;
    }

    /**
     * Gets Command Objective
     * 
     * @return AutoObjective
     */
    public AutoObjective getObjective() {
        return objective;
    }
    
    /**
     * Sets Command Objective
     * 
     * @param objective AutoObjective to override current objective
     */
    public void setObjective(AutoObjective objective) {
        this.objective = objective;
    }

    /**
     * Gets Setpoint
     * 
     * @return setpoint value used by respectively by commands
     */
    public double getSetpoint() {
        return setpoint;
    }
}