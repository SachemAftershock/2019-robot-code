package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Climber Subsystem Class
 * 
 * @author Shreyas Prasad
 */
public class PistonClimber {
    private static PistonClimber instance = new PistonClimber();
    private DoubleSolenoid frontPistons, rearPistons;

    /**
     * Constructor for PistonClimber class
     */
    private PistonClimber() {
        frontPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_FRONT_PISTON_FORWARD_PORT, Constants.CLIMBER_FRONT_PISTON_REVERSE_PORT);
        rearPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_REAR_PISTON_FORWARD_PORT, Constants.CLIMBER_REAR_PISTON_REVERSE_PORT);
        frontPistons.set(Value.kForward);
        rearPistons.set(Value.kForward);

        if(frontPistons.get() == Value.kOff)
            frontPistons.set(Value.kForward);
        if(rearPistons.get() == Value.kOff)
            rearPistons.set(Value.kForward);
    }

    /**
     * Toggles front pistons
     */
    public void toggleFrontPistons() {
        if(frontPistons.get() != Value.kForward) {
            frontPistons.set(Value.kForward);
        } else {
            frontPistons.set(Value.kReverse);
        }
    }

    /**
     * Toggles Rear piston
     */
    public void toggleRearPiston() {
        if(rearPistons.get() != Value.kForward) {
            rearPistons.set(Value.kForward);
        } else {
            rearPistons.set(Value.kReverse);
        }
    }

    /**
     * Retracts all pistons
     */
    public void pullInPistons() {
        rearPistons.set(Value.kForward);
        frontPistons.set(Value.kForward);
    }

    /**
     * Gets instance of PistonClimber singleton
     * 
     * @return Instance of PistonClimber
     */
    public static PistonClimber getInstance() {
        return instance;
    }
}