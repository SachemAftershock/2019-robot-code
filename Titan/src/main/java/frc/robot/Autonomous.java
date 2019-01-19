package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

class Autonomous {
    
    private static Autonomous autoInstance = new Autonomous();
    private Queue<AutoTask> taskList;
    AutoTask target;
    private SWDrive driveBase;
    private Autonomous() {
        taskList = new LinkedList<AutoTask>();
        driveBase = SWDrive.getInstance();
        target = null;
    }

    public void drive() {
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

    public static Autonomous getInstance() {
        return autoInstance;
    }

    public boolean setpointReached() {
        switch(target.getObjective()) {
            case LINEAR:
            case ROTATE:
                return driveBase.setpointReached();
            default:
                return true;
        }
    }
}


