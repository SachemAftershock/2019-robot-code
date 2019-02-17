package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

import frc.robot.Commands.BaseCmd;

abstract class Mechanism {
    private Queue<BaseCmd> commandQueue;
    BaseCmd target;

    public Mechanism() {
        commandQueue = new LinkedList<BaseCmd>();
        target = null;
    }

    public abstract void drive();

    public void push(BaseCmd cmd) {
        commandQueue.add(cmd);
    }

    public BaseCmd pop() {
        return commandQueue.poll();
    }

    public BaseCmd peek() {
        return commandQueue.peek();
    }

    public int size() {
        return commandQueue.size();
    }

    public void flush() {
        commandQueue.clear();
    }
}