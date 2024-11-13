package org.firstinspires.ftc.teamcode;

//Whether to use the arm
#define USE_ARM
//Whether to use the claw
#define USE_CLAW

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;

import java.security.acl.NotOwnerException;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="New_Code_Juju")
public class NotCopiedDrive extends LinearOpMode {
    /**
     * Represents the powers of the motors.
     * **/
    public class MotorPowers {
        private double frontLeft;
        private double frontRight;
        private double backLeft;
        private double backRight;

        /**
         * Default constructor: zero-initialises everything.
         * **/
        public MotorPowers() {
            this(0.f, 0.f, 0.f, 0.f);
        }

        /**
         * Creates a new MotorPowers from the given powers.
         * @param frontLeft The power of the front left motor.
         * @param frontRight The power of the front right motor.
         * @param backLeft The power of the back left motor.
         * @param backRight The power of the back right motor.
         * **/
        public MotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        public double getFrontLeft() {return this.frontLeft;}
        public double getFrontRight() {return this.frontRight;}
        public double getBackLeft() {return this.backLeft;}
        public double getBackRight() {return this.backRight;}

        public void setFrontLeft(double value) {this.frontLeft = value;}
        public void setFrontRight(double value) {this.frontRight = value;}
        public void setBackLeft(double value) {this.backLeft = value;}
        public void setBackRight(double value) {this.backRight = value;}

        /**
         * Multiplies every power by the given factor.
         * @param factor The multiplier to be applied to every power.
         * **/
        public void scale(double factor) {
            this.frontLeft *= factor;
            this.frontRight *= factor;
            this.backLeft *= factor;
            this.backRight *= factor;
        }

        /**
         * Clamps every value to the range [-1.0, 1.0].
         * Not actually sure if this is necessary at all, or if the motors do this automatically.
         * **/
        public void clamp() {
            if (this.frontLeft > 1.0f) this.frontLeft = 1.0f;
            if (this.frontRight > 1.0f) this.frontRight = 1.0f;
            if (this.backLeft > 1.0f) this.backLeft = 1.0f;
            if (this.backRight > 1.0f) this.backRight = 1.0f;

            if (this.frontLeft < -1.0f) this.frontLeft = -1.0f;
            if (this.frontRight < -1.0f) this.frontRight = -1.0f;
            if (this.backLeft < -1.0f) this.backLeft = -1.0f;
            if (this.backRight < -1.0f) this.backRight = -1.0f;
        }

        /**
         * Applies these powers to the robot's motors.
         * **/
        public void apply() {
            NotCopiedDrive.this.frontLeft.setPower(this.frontLeft);
            NotCopiedDrive.this.frontRight.setPower(this.frontRight);
            NotCopiedDrive.this.backLeft.setPower(this.backLeft);
            NotCopiedDrive.this.backRight.setPower(this.backRight);
        }
    }

    #if USE_ARM
    /**
     * Represents powers of the arm.
     * **/
    public class ArmPowers {
        private double elevation;
        private double angle;

        /**
         * Default constructor: zero-initialises everything.
         * **/
        public ArmPowers() {
            this(0.0);
        }

        /**
         * Sets the elevation to the given value without the arm angle changing.
         * @param elevation The power with which to move the arm. Positive means up, negative means down.
         * **/
        public ArmPowers(double elevation) {
            this(elevation, 0.0);
        }

        /**
         * Creates a new ArmPowers from the given powers.
         * @param elevation The power with which to move the arm. Positive means up, negative means down.
         * @param angle The angle to move the arm to. Must be in the range [0.0, 1.0].
         * **/
        public ArmPowers(double elevation, double angle) {
            this.elevation = elevation;
            this.angle = angle;
        }

        public double getElevation() {return this.elevation;}
        public double getAngle() {return this.angle;}

        public void setElevation(double value) {this.elevation = value;}
        public void setAngle(double value) {this.angle = value;}

        /**
         * Clamps both the elevation and angle to their respective ranges.
         * **/
        public void clamp() {
            if (this.elevation > 1.0f) this.elevation = 1.0;
            if (this.angle > 1.0f) this.angle = 1.0;

            if (this.elevation < -1.0f) this.elevation = -1.0;
            if (this.angle < 0.0f) this.angle = 0.0;
        }

        /**
         * Applies these powers to the robot's motors.
         * **/
        public void apply() {
            NotCopiedDrive.this.armAngle.setPower(this.angle);
            NotCopiedDrive.this.armElevation.setPower(this.elevation);
        }
    }
    #endif

    #if USE_CLAW
    /**
     * Represents powers of the claw.
     * **/
    public class ClawPowers {
        private double orientation;
        private double power;

        /**
         * Default constructor: zero-initialises everything.
         * **/
        public ClawPowers() {
            this(0.0);
        }

        /**
         * Sets the intake power to the given power, and maintains the current claw angle state.
         * @param power The intake power.
         * **/
        public ClawPowers(double power) {
            this(NotCopiedDrive.this.clawAngle.getPosition(), power);
        }

        /**
         * Creates a new ClawPowers from the given powers.
         * @param power The intake power.
         * @param orientation The claw orientation.
         * **/
        public ClawPowers(double orientation, double power) {
            this.orientation = orientation;
            this.power = power;
        }

        public double getOrientation() {
            return orientation;
        }
        public double getPower() {
            return power;
        }
        public void setOrientation(double orientation) {
            this.orientation = orientation;
        }
        public void setPower(double power) {
            this.power = power;
        }

        /**
         * Clamps both the orientation and their power to their respective ranges.
         * **/
        public void clamp() {
            if (this.power > 1.0) this.power = 1.0;
            if (this.orientation > 1.0) this.orientation = 1.0;

            if (this.power < -1.0) this.power = -1.0;
            if (this.orientation < 0.0) this.orientation = 0.0;
        }

        /**
         * Applies these powers to the robot's motors.
         * **/
        public void apply() {
            NotCopiedDrive.this.clawIntake.setPower(this.power);
            NotCopiedDrive.this.clawAngle.setPosition(this.orientation);
        }
    }
    #endif

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    #if USE_ARM
    public DcMotor armElevation;
    public DcMotor armAngle;
    #endif

    #if USE_CLAW
    public Servo clawAngle;
    public CRServo clawIntake;
    #endif

    /**
     * Initialises the robot's hardware. Dead simple.
     * **/
    public void initHardware() {
        this.frontLeft = this.hardwareMap.get(DcMotor.class, "drivefl");
        this.backLeft = this.hardwareMap.get(DcMotor.class, "drivebl");
        this.frontRight = this.hardwareMap.get(DcMotor.class, "drivefr");
        this.backRight = this.hardwareMap.get(DcMotor.class, "drivebr");

        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        #if USE_ARM
        this.armElevation = this.hardwareMap.get(DcMotor.class, "actuator");
        this.armAngle = this.hardwareMap.get(DcMotor.class, "elbow");

        this.armElevation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        #endif

        #if USE_CLAW
        this.clawAngle = this.hardwareMap.get(Servo.class, "");
        this.clawIntake = this.hardwareMap.get(CRServo.class, "");
        #endif
    }

    /**
     * Returns the motor powers calculated from the gamepad.
     * @return The motor powers, which can be directly applied to the motors with one function call.
     * **/
    public MotorPowers getMotorPowers() {
        double axial = AutoUtil.twoWayThreshold(-this.gamepad1.left_stick_x, -0.7, 0.7);
        double lateral = AutoUtil.twoWayThreshold(this.gamepad1.left_stick_y, -0.7, 0.7);
        double yaw =AutoUtil.twoWayThreshold(this.gamepad1.right_stick_x, -0.7, 0.7);
        boolean slow = this.gamepad1.a;

        MotorPowers ret = new MotorPowers(
                axial + lateral + yaw,
                axial - lateral - yaw,
                axial - lateral + yaw,
                axial + lateral - yaw
        );
        if (slow) ret.scale(0.25);
        ret.clamp();
        return ret;
    }

    #if USE_ARM
    /**
     * Returns the arm powers calculated from the gamepad.
     * @return The arm powers, which can be directly applied to the motors with one function call.
     * **/
    public ArmPowers getArmPowers() {
        boolean up = this.gamepad1.dpad_up;
        boolean down = this.gamepad1.dpad_down;
        boolean left = this.gamepad1.dpad_left;
        boolean right = this.gamepad1.dpad_right;

        double elevation = AutoUtil.ternaryXOR(up, down);
        double angle = AutoUtil.ternaryXOR(left, right);

        ArmPowers ret = new ArmPowers(elevation, angle);
        ret.clamp();
        return ret;
    }
    #endif

    #if USE_CLAW
    /**
     * Returns the claw powers calculated from the gamepad.
     * @return The claw powers, which can be directly applied to the motors with one function call.
     * **/
    public ClawPowers getClawPowers() {
        double intake = AutoUtil.twoWayThreshold(this.gamepad1.right_stick_x, -0.7, 0.7);
        boolean up = this.gamepad1.left_bumper;
        boolean down = this.gamepad1.right_bumper;

        double position = AutoUtil.ternaryXOR(up, down);
        position = this.clawAngle.getPosition() + position * 0.05;

        ClawPowers ret = new ClawPowers(position, intake);
        ret.clamp();
        return ret;
    }
    #endif

    //NO PASTING
    @Override
    public void runOpMode() throws InterruptedException {
        this.initHardware();
        this.waitForStart();

        while (this.opModeIsActive()) {
            MotorPowers drivePowers = this.getMotorPowers();
            drivePowers.apply();

            #if USE_ARM
            ArmPowers armPowers = this.getArmPowers();
            armPowers.apply();
            #endif

            #if USE_CLAW
            ClawPowers clawPowers = this.getClawPowers();
            clawPowers.apply();
            #endif
        }

        //Brakes the robot.
        new MotorPowers().apply();
        #if USE_ARM
        new ArmPowers().apply();
        #endif
        #if USE_CLAW
        new ClawPowers().apply();
        #endif
    }
}
