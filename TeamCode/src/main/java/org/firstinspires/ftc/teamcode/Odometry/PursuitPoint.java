package org.firstinspires.ftc.teamcode.Odometry;

import java.util.ArrayList;

import RMath.Point;

class PursuitPoint {

    private boolean hasLookahead = false;

    private boolean hasRotation = false;

    private ArrayList<Action> actions = new ArrayList<>();

    public Point position;
    public double rotation;
    public double lookahead;

    public boolean hasLookahead() {
        return hasLookahead;
    }

    public boolean hasRotation() {
        return hasRotation;
    }

    public PursuitPoint(double x, double y) {
        position = new Point(x, y);
    }

    public PursuitPoint(double x, double y, double d, boolean isLookahead) {
        position = new Point(x, y);
        if (isLookahead) {
            hasLookahead = true;
            hasRotation = false;
            lookahead = d;
        } else {
            hasRotation = true;
            hasLookahead = false;
            rotation = d;
        }
    }

    public PursuitPoint(double x, double y, double rotation, double lookahead) {
        position = new Point(x, y);

        this.rotation = rotation;
        this.lookahead = lookahead;

        hasLookahead = true;
        hasRotation = true;
    }

    public ArrayList<Action> getActions() {
        return actions;
    }

    public PursuitPoint(PursuitPoint p) {
        hasLookahead = p.hasLookahead;
        hasRotation = p.hasRotation;
        position = new Point(p.position.x, p.position.y);
        rotation = p.rotation;
        lookahead = p.lookahead;
        actions = p.actions;
    }

    public void addAction(Runnable r, boolean isThread) {
        actions.add(new Action(r, isThread));
    }

    public void addAction(Runnable r, boolean isThread, double moveTolerance) {
        actions.add(new Action(r, isThread, moveTolerance));
    }

    class Action {
        public Runnable action;
        private boolean running = false;
        private boolean started = false;
        private boolean thread;
        public double moveTolerance = Double.NaN;

        public Action(Runnable action, boolean isThread) {
            this.action = action;
            thread = isThread;
        }

        public Action(Runnable action, boolean isThread, double moveTolerance) {
            this.action = action;
            thread = isThread;
            this.moveTolerance = moveTolerance;
        }

        public void run() {
            running = true;
            started = true;
            action.run();
            running = false;
        }

        public boolean isRunning() {
            return running;
        }

        public boolean hasStarted() {
            return started;
        }

        public boolean isThread() {
            return thread;
        }

    }

}
