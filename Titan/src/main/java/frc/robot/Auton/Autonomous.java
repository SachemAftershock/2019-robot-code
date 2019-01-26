package frc.robot.Auton;

import java.util.LinkedList;
import java.util.Queue;
import frc.robot.SWDrive;

import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
    
    private static Autonomous autoInstance = new Autonomous();
    private Queue<AutonBase> taskList;
    AutonBase target;
    Timer autoTimer;
    boolean firstTimer;
    private SWDrive driveBase;

    private Autonomous() {
        taskList = new LinkedList<AutonBase>();
        driveBase = SWDrive.getInstance();
        autoTimer = new Timer();
        firstTimer = true;
        target = null;
    }

    public void drive() {
        if(taskList.size() == 0 && setpointReached()) {
            System.out.println("Autonomous Finished");
            return;
        }

        if(target == null || setpointReached() || autoTimer.hasPeriodPassed(2.0)) {
            target = taskList.poll();
            autoTimer.reset();

            System.out.println("New Task: " + target.getTask());
        }

        switch(target.getTask()) {
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
                System.out.println("Unknown Autonomous mode found: " + target.getTask());
        }
    }

    public void queueObjective(AutonBase task) {
        taskList.add(task);
    }

    public static Autonomous getInstance() {
        return autoInstance;
    }

    public boolean setpointReached() {
        switch(target.getTask()) {
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


