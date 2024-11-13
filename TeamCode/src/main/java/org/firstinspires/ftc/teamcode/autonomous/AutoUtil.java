package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

public class AutoUtil {
    public static class OpModeInterruptedException extends Exception {
        public OpModeInterruptedException() {
            super("OpMode interrupted during sleep!");
        }

        public OpModeInterruptedException(String msg) {
            super("OpMode interrupted during sleep!\n" + msg);
        }
    }

    /**
     * Keeps track of a number.
     * Basically a fancy int.
     * **/
    public static class Counter {
        private int counts;

        public Counter(int counts) {
            this.counts = counts;
        }

        public void increment() {
            ++this.counts;
        }

        public void decrement() {
            --this.counts;
        }

        public int getCounts() {
            return this.counts;
        }

        @Override
        public String toString() {
            return "Counter(" + this.counts + ")";
        }
    }

    /**
     * Literally just holds a value.
     * Surprisingly useful though. Basically the closest that Java has to a pointer.
     * **/
    public static class Value<T> {
        public T value;

        Value(T value) {this.value = value;}
        Value() {this.value = null;}
    }

    /**
     * Nobody uses iterative OpModes :skull:
     * **/
    private static LinearOpMode opMode = null;

    /**
     * Sets the currently running OpMode.
     * This is useful for throwing exceptions if the OpMode is stopped early.
     * @param opMode The opMode to set. Must be linear because iterative OpModes apparently don't work that way.
     * **/
    public static void setOpMode(LinearOpMode opMode) {
        AutoUtil.opMode = opMode;
    }

    /**
     * Gets the currently running OpMode.
     * **/
    public static LinearOpMode getOpMode() {
        return AutoUtil.opMode;
    }

    /**
     * Returns true if the OpMode has been terminated.
     * **/
    public static boolean shouldStop() {
        return !(AutoUtil.opMode.opModeIsActive() || AutoUtil.opMode.opModeInInit());
    }

    /**
     * Waits on the given counter.
     * @param counter The counter to wait on (until it reaches 0).
     * @param timeoutMillis The timeout in milliseconds. If 0, waits for forever.
     * @return true if the function timed out without the counter being set, false otherwise.
     * @throws OpModeInterruptedException Throws if the OpMode is interrupted.
     * **/
    public static boolean waitOnCounter(@NonNull Counter counter, long timeoutMillis) throws OpModeInterruptedException {
        if (timeoutMillis == 0) RobotLog.w("Waiting on counter " + counter + " without a timeout. This might freeze.");
        long deadline = timeoutMillis == 0 ? -1 : System.currentTimeMillis() + timeoutMillis;
        while (System.currentTimeMillis() != deadline) {
            if (AutoUtil.shouldStop())
                throw new OpModeInterruptedException("AutoUtil::waitOnCounter(" + counter + ", " + timeoutMillis + ")");
            if (counter.getCounts() == 0) return false;
        }
        return true;
    }

    /**
     * Waits for the given time.
     * @param timeoutMillis The time to wait for, in milliseconds.
     * @throws OpModeInterruptedException Throws if the OpMode is interrupted.
     * **/
    public static void safeWait(long timeoutMillis) throws OpModeInterruptedException {
        long deadline = System.currentTimeMillis() + timeoutMillis;
        while (System.currentTimeMillis() != deadline) if (AutoUtil.shouldStop())
            throw new OpModeInterruptedException("AutoUtil::wait(" + timeoutMillis + ")");
    }

    /**
     * Compares two booleans. Returns a non-zero value if only one value is true, but the specific value depends on which one is true.
     * Mostly useful for input measurement.
     * @param positive Returns 1.0 if it is the only one that is true.
     * @param negative Returns -1.0 if it is the only one that is true.
     * @return See above.
     * **/
    public static double ternaryXOR(boolean positive, boolean negative) {
        if (positive == negative) return 0.0;
        else if (positive) return 1.0;
        else return -1.0;
    }

    /**
     * If lower < in < upper, return 0. Otherwise, return in.
     * @param in The input.
     * @param lower The lower bound.
     * @param upper The upper bound.
     * **/
    public static double twoWayThreshold(double in, double lower, double upper) {
        if (in > lower && in < upper) return 0;
        return in;
    }
}
