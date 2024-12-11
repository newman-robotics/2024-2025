package org.firstinspires.ftc.teamcode.deprecated;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

/**
 * Deprecated versions of AutoUtil (and perhaps other classes).
 * Do not use these unless it is absolutely necessary; better alternatives always exist.
 * **/
@Deprecated
public class DeprecatedUtil {
    /**
     * The entire hardware for the robot. Singleton class.
     * NOTICE: This is the deprecated version that only works for the old arm.
     * @see OldHardware
     * @deprecated This class only functions for the old arm.
     * **/
    public static class OldHardware {
        private static OldHardware instance = null;

        private final DcMotor frontLeft, frontRight, backLeft, backRight;
        private final DcMotor armElevation, armElbow;
        private final CRServo clawIntake;
        private final Servo clawWrist;

        private OldHardware(HardwareMap map) {
            this.frontLeft = map.get(DcMotor.class, GlobalConstants.FRONT_LEFT_MOTOR_NAME);
            this.frontRight = map.get(DcMotor.class, GlobalConstants.FRONT_RIGHT_MOTOR_NAME);
            this.backLeft = map.get(DcMotor.class, GlobalConstants.BACK_LEFT_MOTOR_NAME);
            this.backRight = map.get(DcMotor.class, GlobalConstants.BACK_RIGHT_MOTOR_NAME);

            this.armElevation = map.get(DcMotor.class, GlobalConstants.ARM_VERTICAL_MOTOR_NAME);
            this.armElbow = map.get(DcMotor.class, GlobalConstants.OLD_ARM_ELBOW_MOTOR_NAME);

            if (GlobalConstants.CLAW_IS_INSTALLED) {
                this.clawWrist = map.get(Servo.class, GlobalConstants.OLD_CLAW_WRIST_MOTOR_NAME);
                this.clawIntake = map.get(CRServo.class, GlobalConstants.OLD_CLAW_INTAKE_MOTOR_NAME);
            } else {
                this.clawWrist = null;
                this.clawIntake = null;
            }

            this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.armElevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.armElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.armElbow.setPower(GlobalConstants.ARM_ELBOW_SPEED);
            this.armElbow.setTargetPosition(0);
            this.armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        /**
         * Constructs and returns the hardware.
         * @param map The hardware map of the invoking OpMode.
         * @return Hardware.
         * **/
        public static OldHardware init(HardwareMap map) {
            OldHardware.instance = new OldHardware(map);
            return OldHardware.instance;
        }

        /**
         * Gets the hardware.
         * @return Hardware.
         * **/
        @Nullable
        public static OldHardware get() {
            return OldHardware.instance;
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

            if (GlobalConstants.CLAW_IS_INSTALLED) {
                this.clawIntake.setPower(0);
            }
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
         * @param elbow Elbow motor relative position.
         * **/
        public void setArmPowers(double elevation, int elbow) {
            this.armElevation.setPower(elevation);
            this.armElbow.setTargetPosition(this.armElbow.getCurrentPosition() + elbow);
        }

        /**
         * Sets the powers for the claw. Also takes care of clamping.
         * @param wrist Wrist motor relative position.
         * @param intake Intake motor power.
         * **/
        public void setClawPowers(double wrist, double intake) {
            if (GlobalConstants.CLAW_IS_INSTALLED) {
                this.clawWrist.setPosition(this.clawWrist.getPosition() + wrist);
                this.clawIntake.setPower(AutoUtil.clamp(intake, -1, 1));
            } else RobotLog.w("AutoUtil::Hardware::setClawPowers() called, but claw is not installed!");
        }
    }

    /**
     * The entire hardware for the robot. Singleton class.
     * Why is this in AutoUtil and not its own dedicated class, I hear you ask?
     * I have no clue.
     * @apiNote This is the new version of the hardware. The old version can be found at {@link OldHardware}
     * **/
    public static class Hardware {
        private static Hardware instance = null;

        //drivetrain
        private final DcMotor frontLeft, frontRight, backLeft, backRight;
        private final DcMotor lowerArm, armElbow;
        private final CRServo upperArm;
        private final Servo claw;

        private Hardware(HardwareMap map) {
            this.frontLeft = map.get(DcMotor.class, GlobalConstants.FRONT_LEFT_MOTOR_NAME);
            this.frontRight = map.get(DcMotor.class, GlobalConstants.FRONT_RIGHT_MOTOR_NAME);
            this.backLeft = map.get(DcMotor.class, GlobalConstants.BACK_LEFT_MOTOR_NAME);
            this.backRight = map.get(DcMotor.class, GlobalConstants.BACK_RIGHT_MOTOR_NAME);

            this.lowerArm = map.get(DcMotor.class, GlobalConstants.ARM_VERTICAL_MOTOR_NAME);
            this.armElbow = map.get(DcMotor.class, GlobalConstants.ARM_ELBOW_MOTOR_NAME);

            this.upperArm = map.get(CRServo.class, GlobalConstants.ARM_VERTICAL_SERVO_MOTOR_NAME);
            this.claw = map.get(Servo.class, GlobalConstants.CLAW_MOTOR_NAME);

            this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.armElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            this.armElbow.setPower(GlobalConstants.ARM_ELBOW_SPEED);
            this.armElbow.setTargetPosition(0);
            this.armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
         * **/
        @Nullable
        public static Hardware get() {
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

            this.upperArm.setPower(0);
            this.lowerArm.setPower(0);
            this.armElbow.setPower(0);
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
         * @param lower Lower elevation (actuator) motor power.
         * @param upper Upper elevation (servo) motor power.
         * @param elbow Elbow motor relative position.
         * **/
        public void setArmPowers(double lower, double upper, int elbow) {
            this.lowerArm.setPower(lower);
            this.upperArm.setPower(upper);
            this.armElbow.setTargetPosition(this.armElbow.getCurrentPosition() + elbow);
        }

        public void setArmPowers2(double upper, int elbow) {
            //this.lowerArm.setPower(lower); //Motor (Originally an actuator)
            this.upperArm.setPower(upper); //CRservo
            this.armElbow.setTargetPosition(this.armElbow.getCurrentPosition() + elbow); //Motor
        }

        /**
         * Sets the powers for the claw. Also takes care of clamping.
         * @param claw Claw motor relative position.
         * **/
        public void setClawPowers(double claw) {
            this.claw.setPosition(this.claw.getPosition() + claw); //servo
        }
    }
}
