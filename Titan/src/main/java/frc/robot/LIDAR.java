package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class LIDAR {
    private Counter counter;
    
    public LIDAR(DigitalSource port) {
        counter = new Counter(port);
        counter.setMaxPeriod(1.0);
        counter.setSemiPeriodMode(true);
        counter.reset();
    }

    public double getDistanceCm() {
        return counter.getPeriod() * 1000000.0 / 10.0;
    }
    public double getDistanceIn() {
        return getDistanceCm() * .393700787;
    }
}