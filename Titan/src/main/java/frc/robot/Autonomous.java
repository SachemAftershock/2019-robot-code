package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;

class Autonomous {
    
    private static Autonomous autoInstance = new Autonomous();
    private Queue<AutoTask> taskList;
    AutoTask target;
    Timer autoTimer;
    boolean firstTimer;
    private SWDrive driveBase;

    private Autonomous() {
        taskList = new LinkedList<AutoTask>();
        driveBase = SWDrive.getInstance();
        autoTimer = new Timer();
        firstTimer = true;
        target = null;
    }

    public void drive() {
        if(taskList.size() == 0 && setpointReached()) {
            return;
        }

        if(target == null || setpointReached()) {
            target = taskList.poll();
        }

        switch(target.getObjective()) {
            case LINEAR:
                driveBase.autoDrive(target.getSetpoint());
                break;
            case ROTATE:
                driveBase.autoRotate(target.getSetpoint());
                break;
            //case WAIT:
              //  if(firstTimer) {
                //    autoTimer.reset();
                  //  firstTimer = false;
                //}
               // break; 
            default:
                System.out.println("Unknown Autonomous mode found: " + target.getObjective());
        }
    }

    public void queueLinearDistance(Distance dist, double distance) {
        taskList.add(new AutoTask(AutoObjective.LINEAR, distance * dist.getConversionFactor()));
    }

    public void queueAutoRotate(Rotation rot, double theta) {
        taskList.add(new AutoTask(AutoObjective.ROTATE, theta * rot.getDirection()));
    }

    //public void queueWait(double seconds) {
    //    taskList.add(new AutoTask(AutoObjective.WAIT, seconds));
    //}

    public static Autonomous getInstance() {
        return autoInstance;
    }

    public boolean setpointReached() {
        switch(target.getObjective()) {
            case LINEAR:
            case ROTATE:
                return driveBase.setpointReached();
            //case WAIT:
            //    if(autoTimer.get() > target.getSetpoint()) {
            //        firstTimer = true;
             //       return true;
               // }
                //return false;
            default:
                return true;
        }
    }
}


