package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

import frc.robot.Commands.BaseCmd;

/**
 * Abstract Mechanism Class to provide each Subsystem with a Command Queue
 * 
 * @author Rohan Bapat
 */
abstract class Mechanism {
    private Queue<BaseCmd> commandQueue;
    BaseCmd target;

    /**
     * Mechanism Constructor
     */
    public Mechanism() {
        commandQueue = new LinkedList<BaseCmd>();
        target = null;
    }

    public abstract void drive();

    /**
     * Add Command to end of Command Queue
     * 
     * @param cmd Command to push to add to Command Queue
     */
    public void push(BaseCmd cmd) {
        commandQueue.add(cmd);
    }

    /**
     * Pop head Command in Command Queue
     * 
     * @return Command removed from Queue
     */
    public BaseCmd pop() {
        return commandQueue.poll();
    }

    /**
     * Get head of Command Queue
     * 
     * @return first Command in Queue
     */
    public BaseCmd peek() {
        return commandQueue.peek();
    }

    /**
     * Get size of Queue
     * 
     * @return size of Queue
     */
    public int size() {
        return commandQueue.size();
    }

    /**
     * Clears all Commands in Command Queue
     */
    public void flush() {
        commandQueue.clear();
        target = null;
    }
}