package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class PistonClimber extends Mechanism {
    private static PistonClimber instance = new PistonClimber();
    private DoubleSolenoid frontPistons, rearPistons;

    private PistonClimber() {
        frontPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_FRONT_PISTON_FORWARD_PORT, Constants.CLIMBER_FRONT_PISTON_REVERSE_PORT);
        rearPistons = new DoubleSolenoid(Constants.CLIMBER_PCM_PORT, Constants.CLIMBER_REAR_PISTON_FORWARD_PORT, Constants.CLIMBER_REAR_PISTON_REVERSE_PORT);
        frontPistons.set(Value.kForward);
        rearPistons.set(Value.kForward);
        System.out.println("Front Piston: " + frontPistons.get() + ", Back Piston: " + rearPistons.get());
        if(frontPistons.get() == Value.kOff)
            frontPistons.set(Value.kForward);

        if(rearPistons.get() == Value.kOff)
            rearPistons.set(Value.kForward);
        System.out.println("Front Piston: " + frontPistons.get() + ", Back Piston: " + rearPistons.get());
    }

    public void drive() {
        //Not really anything to put here, doubt we're going to be autonomously using this thingy
    }

    public void toggleFrontPistons() {
        System.out.print("Toggling front pistons");
        if(frontPistons.get() != Value.kForward) {
            frontPistons.set(Value.kForward);
        } else {
            frontPistons.set(Value.kReverse);
        }
        System.out.println(" " + frontPistons.get());
    }
    public void toggleRearPiston() {
        System.out.print("Toggling rear piston");
        if(rearPistons.get() != Value.kForward) {
            rearPistons.set(Value.kForward);
        } else {
            rearPistons.set(Value.kReverse);
        }
        System.out.println(" " + rearPistons.get());
    }

    public void pullInPistons() {
        rearPistons.set(Value.kForward);
        frontPistons.set(Value.kForward);
    }

    public static PistonClimber getInstance() {
        return instance;
    }
}