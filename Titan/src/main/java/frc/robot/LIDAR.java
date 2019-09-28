package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

/**
 * LIDAR Object Class
 * 
 * @author Shreyas Prasad
 */
public class LIDAR {
    private Counter counter;
    
    /**
     * LIDAR Class Constructor
     * 
     * @param port DigitalInput Object corresponding to the LIDAR PWM Port
     */
    public LIDAR(DigitalSource port) {
        counter = new Counter(port);
        counter.setMaxPeriod(1.0);
        counter.setSemiPeriodMode(true);
        counter.reset();
    }

    /**
     * Get current LIDAR Distance in cm
     * 
     * @return current LIDAR Distance reading in cm
     */
    public double getDistanceCm() {
        return counter.getPeriod() * 1000000.0 / 10.0;
    }
}