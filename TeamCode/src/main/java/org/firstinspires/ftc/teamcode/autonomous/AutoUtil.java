package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
     * The entire hardware for the robot. Singleton class.
     * Why is this in AutoUtil and not its own dedicated class, I hear you ask?
     * I have no clue.
     * **/
    public static class Hardware {
        private static Hardware instance = null;

        private final DcMotor frontLeft, frontRight, backLeft, backRight;
        private final DcMotor armElevation, armElbow;
        private final Servo clawWrist;
        private final CRServo clawIntake;

        private Hardware(HardwareMap map) {
            this.frontLeft = map.get(DcMotor.class, GlobalConstants.FRONT_LEFT_MOTOR_NAME);
            this.frontRight = map.get(DcMotor.class, GlobalConstants.FRONT_RIGHT_MOTOR_NAME);
            this.backLeft = map.get(DcMotor.class, GlobalConstants.BACK_LEFT_MOTOR_NAME);
            this.backRight = map.get(DcMotor.class, GlobalConstants.BACK_RIGHT_MOTOR_NAME);

            this.armElevation = map.get(DcMotor.class, GlobalConstants.ARM_VERTICAL_MOTOR_NAME);
            this.armElbow = map.get(DcMotor.class, GlobalConstants.ARM_ELBOW_MOTOR_NAME);

            this.clawWrist = map.get(Servo.class, GlobalConstants.CLAW_WRIST_MOTOR_NAME);
            this.clawIntake = map.get(CRServo.class, GlobalConstants.CLAW_INTAKE_MOTOR_NAME);

            this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.armElevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.armElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /**
         * Constructs and returns the hardware.
         * @param map The hardware map of the invoking OpMode.
         * @return Hardware.
         * **/
        public static Hardware init(HardwareMap map) {
            Hardware.instance = new Hardware(map);
            return Hardware.instance;
        }

        /**
         * Gets the hardware.
         * @return Hardware.
         * @throws IllegalAccessException Throws if the Hardware instance has not been initialised.
         * **/
        public static Hardware get() throws IllegalAccessException {
            if (Hardware.instance == null) throw new IllegalAccessException("Hardware requested, but not initialised!");
            return Hardware.instance;
        }

        /**
         * Stops everything from moving.
         * **/
        public void zeroOut() {
            this.frontLeft.setPower(0);
            this.frontRight.setPower(0);
            this.backLeft.setPower(0);
            this.backRight.setPower(0);

            this.armElevation.setPower(0);
            this.armElbow.setPower(0);

            this.clawIntake.setPower(0);
        }

        /**
         * Sets the powers for the drivetrain. Also takes care of clamping.
         * @param frontLeft Front left motor power.
         * @param frontRight Front right motor power.
         * @param backLeft Back left motor power (note: reversed due to a hardware issue).
         * @param backRight Back right motor power.
         * **/
        public void setDrivetrainPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeft.setPower(AutoUtil.clamp(frontLeft, -1, 1));
            this.frontRight.setPower(AutoUtil.clamp(frontRight, -1, 1));
            this.backLeft.setPower(AutoUtil.clamp(backLeft, -1, 1));
            this.backRight.setPower(AutoUtil.clamp(backRight, -1, 1));
        }

        /**
         * Sets the powers for the arm. Also takes care of clamping.
         * @param elevation Elevation motor power.
         * @param elbow Elbow motor power.
         * **/
        public void setArmPowers(double elevation, double elbow) {
            this.armElevation.setPower(elevation);
            this.armElbow.setPower(elbow);
        }

        /**
         * Sets the powers for the claw. Also takes care of clamping.
         * @param wrist Wrist motor relative position (i.e. relative to where it is now).
         * @param intake Intake motor power.
         * **/
        public void setClawPowers(double wrist, double intake) {
            this.clawWrist.setPosition(AutoUtil.clamp(this.clawWrist.getPosition() + wrist, 0, 1));
            this.clawIntake.setPower(AutoUtil.clamp(intake, -1, 1));
        }

        public void setClawWristAbsolutePosition(double position) {
            this.clawWrist.setPosition(position);
        }
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
     * If lower < in < upper, return 0. Otherwise, return in. (Rough) opposite of clamp.
     * @param in The input.
     * @param lower The lower bound.
     * @param upper The upper bound.
     * **/
    public static double twoWayThreshold(double in, double lower, double upper) {
        if (in > lower && in < upper) return 0;
        return in;
    }

    /**
     * Clamps in to the range [lower, upper].
     * **/
    public static double clamp(double in, double lower, double upper) {
        return Math.min(Math.max(in, lower), upper);
    }
}
