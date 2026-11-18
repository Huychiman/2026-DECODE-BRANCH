package org.firstinspires.ftc.teamcode.logic;

import java.util.ArrayList;
import java.util.List;

/**
 * Fires a list of sorter slots one after another:
 * rotate sorter -> wait for flywheel at speed -> kick -> retract -> next.
 *
 * <p>Pure state machine driven by {@link #update}; the hardware layer just copies
 * {@link #isKickerFiring()} and {@link #getSorterSlot()} to the servos. A spin-up timeout makes sure
 * a slow or faulty flywheel can never stall autonomous.
 */
public final class ShotSequencer {
    public static final class Timing {
        public double sorterSettleSeconds = 0.40; // servo travel between adjacent slots
        public double feedSeconds = 0.25; // kicker pushing artifact into flywheel
        public double retractSeconds = 0.30; // kicker back out of the way before sorter moves
        public double spinUpTimeoutSeconds = 2.0;
    }

    public enum State {
        IDLE,
        ROTATING,
        WAITING_FOR_FLYWHEEL,
        FEEDING,
        RETRACTING,
        DONE
    }

    private final Timing timing;
    private final List<Integer> queue = new ArrayList<>();
    private State state = State.IDLE;
    private double stateTime;
    private int sorterSlot;
    private int rotateSlots = 1;
    private int shotsFired;
    private int timedOutShots;

    public ShotSequencer(Timing timing, int initialSlot) {
        this.timing = timing;
        this.sorterSlot = initialSlot;
    }

    /** Starts a new volley. Cancels any volley in progress. */
    public void start(List<Integer> slotOrder) {
        queue.clear();
        queue.addAll(slotOrder);
        shotsFired = 0;
        timedOutShots = 0;
        enter(queue.isEmpty() ? State.DONE : State.ROTATING);
        if (!queue.isEmpty() && queue.get(0) == sorterSlot) enter(State.WAITING_FOR_FLYWHEEL);
    }

    public void cancel() {
        queue.clear();
        enter(State.IDLE);
    }

    /** Lets the driver move the sorter manually while no volley is running. */
    public void setSorterSlot(int slot) {
        if (!isBusy()) sorterSlot = slot;
    }

    public void update(double dt, boolean flywheelReady) {
        stateTime += dt;
        switch (state) {
            case ROTATING:
                if (sorterSlot != queue.get(0)) {
                    rotateSlots = Math.abs(queue.get(0) - sorterSlot);
                    sorterSlot = queue.get(0);
                }
                // A two-slot move (0 <-> 2) takes twice as long as an adjacent one.
                if (stateTime >= timing.sorterSettleSeconds * Math.max(1, rotateSlots)) {
                    enter(State.WAITING_FOR_FLYWHEEL);
                }
                break;
            case WAITING_FOR_FLYWHEEL:
                if (flywheelReady) {
                    enter(State.FEEDING);
                } else if (stateTime >= timing.spinUpTimeoutSeconds) {
                    timedOutShots++;
                    enter(State.FEEDING);
                }
                break;
            case FEEDING:
                if (stateTime >= timing.feedSeconds) {
                    shotsFired++;
                    queue.remove(0);
                    enter(State.RETRACTING);
                }
                break;
            case RETRACTING:
                if (stateTime >= timing.retractSeconds) {
                    if (queue.isEmpty()) enter(State.DONE);
                    else if (queue.get(0) == sorterSlot) enter(State.WAITING_FOR_FLYWHEEL);
                    else enter(State.ROTATING);
                }
                break;
            default:
                break;
        }
    }

    private void enter(State s) {
        state = s;
        stateTime = 0;
    }

    public boolean isKickerFiring() {
        return state == State.FEEDING;
    }

    public int getSorterSlot() {
        return sorterSlot;
    }

    public boolean isBusy() {
        return state != State.IDLE && state != State.DONE;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public State getState() {
        return state;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    /** Shots fed without the flywheel reporting ready (usually means kV/kP need tuning). */
    public int getTimedOutShots() {
        return timedOutShots;
    }
}
