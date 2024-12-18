package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Various static functions.
 * (When I say various, I mean there's a little bit of everything in here.)
 * **/
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
     * Allows for easier addition of telemetry via function chaining. Singleton class.
     * **/
    public static class ChainTelemetry {
        private static ChainTelemetry instance;

        private final Telemetry telemetry;

        private ChainTelemetry(Telemetry telemetry) {this.telemetry = telemetry;}

        /**
         * Constructs and returns the chain telemetry.
         * @param telemetry The telemetry of the invoking OpMode.
         * @return ChainTelemetry.
         * **/
        public static ChainTelemetry init(Telemetry telemetry) {
            ChainTelemetry.instance = new ChainTelemetry(telemetry);
            return ChainTelemetry.instance;
        }

        /**
         * Gets the ChainTelemetry.
         * @return ChainTelemetry, or null if it hasn't been constructed.
         * **/
        @Nullable
        public static ChainTelemetry get() {
            return ChainTelemetry.instance;
        }

        /**
         * Asserts that the ChainTelemetry is not null, then gets it.
         * @return ChainTelemetry.
         * **/
        @NonNull
        public static ChainTelemetry assertAndGet() {
            assert ChainTelemetry.instance != null;
            return ChainTelemetry.instance;
        }

        public Telemetry getTelemetry() {
            return telemetry;
        }

        /**
         * See {@link Telemetry#addLine(String) Telemetry.addLine(String)}.
         * **/
        public ChainTelemetry add(String caption) {
            this.telemetry.addLine(caption);
            return this;
        }

        /**
         * See {@link Telemetry#addData(String, Object) Telemetry.addData(String, Object)}.
         * **/
        public ChainTelemetry add(String caption, Object object) {
            this.telemetry.addData(caption, object);
            return this;
        }

        /**
         * See {@link Telemetry#addData(String, Func) Telemetry.addData(String, Func)}.
         * **/
        public <T> ChainTelemetry add(String caption, Func<T> producer) {
            this.telemetry.addData(caption, producer);
            return this;
        }

        /**
         * See {@link Telemetry#addData(String, String, Object...) Telemetry.addData(String, String, Object...)}.
         * **/
        public ChainTelemetry add(String caption, String format, Object... args) {
            this.telemetry.addData(caption, format, args);
            return this;
        }

        /**
         * See {@link Telemetry#addData(String, String, Func) Telemetry.addData(String, String, Func)}.
         * **/
        public <T> ChainTelemetry add(String caption, String format, Func<T> producer) {
            this.telemetry.addData(caption, format, producer);
            return this;
        }

        /**
         * See {@link Telemetry#addLine() Telemetry.addLine()}.
         * **/
        public ChainTelemetry addLine() {
            this.telemetry.addLine();
            return this;
        }

        /**
         * See {@link Telemetry#update() Telemetry.update()}.
         * **/
        public void update() {
            this.telemetry.update();
        }
    }

    /**
     * Represents a four-wheeled drivetrain using Mecanum wheels.
     * Singleton class.
     * **/
    public static class Drivetrain {
        private static Drivetrain instance = null;

        public static Drivetrain initAndGet(HardwareMap map) {
            Drivetrain.instance = new Drivetrain(map);
            return Drivetrain.instance;
        }

        public static Drivetrain assertAndGet() {
            assert Drivetrain.instance != null;
            return Drivetrain.instance;
        }

        private final DcMotor frontLeft;
        private final DcMotor frontRight;
        private final DcMotor backLeft;
        private final DcMotor backRight;

        private Drivetrain(HardwareMap map) {
            this.frontLeft = map.get(DcMotor.class, GlobalConstants.FRONT_LEFT_MOTOR_NAME);
            this.frontRight = map.get(DcMotor.class, GlobalConstants.FRONT_RIGHT_MOTOR_NAME);
            this.backLeft = map.get(DcMotor.class, GlobalConstants.BACK_LEFT_MOTOR_NAME);
            this.backRight = map.get(DcMotor.class, GlobalConstants.BACK_RIGHT_MOTOR_NAME);

            this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void setPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeft.setPower(frontLeft);
            this.frontRight.setPower(frontRight);
            this.backLeft.setPower(backLeft);
            this.backRight.setPower(backRight);
        }

        public void setFromGamepad() {
            double axial = -parseGamepadInputAsDouble(GlobalConstants.AXIAL);
            double lateral = parseGamepadInputAsDouble(GlobalConstants.LATERAL);
            double yaw = parseGamepadInputAsDouble(GlobalConstants.YAW);

            if (parseGamepadInputAsBoolean(GlobalConstants.SLOW)) {
                axial *= GlobalConstants.SLOW_FACTOR;
                lateral *= GlobalConstants.SLOW_FACTOR;
                yaw *= GlobalConstants.SLOW_FACTOR;
            }

            this.setPowers(
                    axial + lateral + yaw,
                    axial - lateral - yaw,
                    axial - lateral + yaw,
                    axial + lateral - yaw
            );
        }
    }

    /**
     * Nobody uses iterative OpModes :skull:
     * **/
    private static LinearOpMode opMode = null;

    /**
     * Sets the currently running OpMode, and initialises AutoUtil's member singletons.
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
     * Gets an input from AutoUtil.opMode.gamepad1.
     * @param input The input to measure.
     * @return The raw value if the input is a double, or value ? 1.0 : 0.0 if the value is a boolean.
     * **/
    public static double parseGamepadInputAsDouble(GlobalConstants.GamepadInput input) {
        switch (input) {
            case LEFT_STICK_X: return AutoUtil.opMode.gamepad1.left_stick_x;
            case LEFT_STICK_Y: return AutoUtil.opMode.gamepad1.left_stick_y;
            case RIGHT_STICK_X: return AutoUtil.opMode.gamepad1.right_stick_x;
            case RIGHT_STICK_Y: return AutoUtil.opMode.gamepad1.right_stick_y;
            case BUTTON_A: return AutoUtil.opMode.gamepad1.a ? 1 : 0;
            case BUTTON_B: return AutoUtil.opMode.gamepad1.b ? 1 : 0;
            case BUTTON_X: return AutoUtil.opMode.gamepad1.x ? 1 : 0;
            case BUTTON_Y: return AutoUtil.opMode.gamepad1.y ? 1 : 0;
            case DPAD_UP: return AutoUtil.opMode.gamepad1.dpad_up ? 1 : 0;
            case DPAD_DOWN: return AutoUtil.opMode.gamepad1.dpad_down ? 1 : 0;
            case DPAD_LEFT: return AutoUtil.opMode.gamepad1.dpad_left ? 1 : 0;
            case DPAD_RIGHT: return AutoUtil.opMode.gamepad1.dpad_right ? 1 : 0;
            case LEFT_TRIGGER: return AutoUtil.opMode.gamepad1.left_trigger;
            case RIGHT_TRIGGER: return AutoUtil.opMode.gamepad1.right_trigger;
            case LEFT_BUMPER: return AutoUtil.opMode.gamepad1.left_bumper ? 1 : 0;
            case RIGHT_BUMPER: return AutoUtil.opMode.gamepad1.right_bumper ? 1 : 0;
            case LEFT_STICK_BUTTON: return AutoUtil.opMode.gamepad1.left_stick_button ? 1 : 0;
            case RIGHT_STICK_BUTTON: return AutoUtil.opMode.gamepad1.right_stick_button ? 1 : 0;
            default: throw new IllegalArgumentException("Illegal GlobalConstants.GamepadInput " + input + "!");
        }
    }

    /**
     * Gets an input from AutoUtil.opMode.gamepad1.
     * @param input The input to measure.
     * @return The raw value if the input is a boolean, or value > GlobalConstants.GAMEPAD_THRESHOLD if the value is a double.
     * **/
    public static boolean parseGamepadInputAsBoolean(GlobalConstants.GamepadInput input) {
        switch (input) {
            case LEFT_STICK_X: return AutoUtil.opMode.gamepad1.left_stick_x > GlobalConstants.GAMEPAD_THRESHOLD;
            case LEFT_STICK_Y: return AutoUtil.opMode.gamepad1.left_stick_y > GlobalConstants.GAMEPAD_THRESHOLD;
            case RIGHT_STICK_X: return AutoUtil.opMode.gamepad1.right_stick_x > GlobalConstants.GAMEPAD_THRESHOLD;
            case RIGHT_STICK_Y: return AutoUtil.opMode.gamepad1.right_stick_y > GlobalConstants.GAMEPAD_THRESHOLD;
            case BUTTON_A: return AutoUtil.opMode.gamepad1.a;
            case BUTTON_B: return AutoUtil.opMode.gamepad1.b;
            case BUTTON_X: return AutoUtil.opMode.gamepad1.x;
            case BUTTON_Y: return AutoUtil.opMode.gamepad1.y;
            case DPAD_UP: return AutoUtil.opMode.gamepad1.dpad_up;
            case DPAD_DOWN: return AutoUtil.opMode.gamepad1.dpad_down;
            case DPAD_LEFT: return AutoUtil.opMode.gamepad1.dpad_left;
            case DPAD_RIGHT: return AutoUtil.opMode.gamepad1.dpad_right;
            case LEFT_TRIGGER: return AutoUtil.opMode.gamepad1.left_trigger > GlobalConstants.GAMEPAD_THRESHOLD;
            case RIGHT_TRIGGER: return AutoUtil.opMode.gamepad1.right_trigger > GlobalConstants.GAMEPAD_THRESHOLD;
            case LEFT_BUMPER: return AutoUtil.opMode.gamepad1.left_bumper;
            case RIGHT_BUMPER: return AutoUtil.opMode.gamepad1.right_bumper;
            case LEFT_STICK_BUTTON: return AutoUtil.opMode.gamepad1.left_stick_button;
            case RIGHT_STICK_BUTTON: return AutoUtil.opMode.gamepad1.right_stick_button;
            default: throw new IllegalArgumentException("Illegal GlobalConstants.GamepadInput " + input + "!");
        }
    }

    /**
     * Returns true if there is more than one true value in vals. Otherwise, returns false.
     * **/
    public static boolean notMoreThanOne(boolean... vals) {
        boolean oneFound = false;
        for (boolean val : vals) if (val) {
            if (oneFound) return false;
            oneFound = true;
        }
        return true;
    }

    /**
     * Clamps the input to be greater than lower and less than upper.
     * Specifically, if input.compareTo(upper) > 0, upper is returned, and if input.compareTo(lower) < 0, lower is returned.
     * **/
    public static <T extends Comparable<T>> T clamp(T input, T lower, T upper) {
        if (input.compareTo(upper) > 0) return upper;
        if (input.compareTo(lower) < 0) return lower;
        return input;
    }
}
