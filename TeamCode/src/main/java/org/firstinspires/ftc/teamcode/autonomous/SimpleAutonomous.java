package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.channels.InterruptedByTimeoutException;

@Autonomous(name="SimpleAutonomous")
public class SimpleAutonomous extends LinearOpMode {
    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        ROTLEFT,
        ROTRIGHT
    }

    DcMotor frontLeft, frontRight, backLeft, backRight;

    /**
     * Moves the robot for the specified time.
     * @param direction The direction to move the robot in.
     * @param millis The time to move the robot for, in milliseconds.
     * @param speed The speed at which to move the robot.
     * @throws AutoUtil.OpModeInterruptedException Throws if the OpMode is interrupted or deactivates.
     * **/
    public void move(Direction direction, long millis, float speed) throws AutoUtil.OpModeInterruptedException {
        switch (direction) {
            case FORWARD:
                this.frontLeft.setPower(speed);
                this.frontRight.setPower(speed);
                this.backLeft.setPower(speed);
                this.backRight.setPower(speed);
                break;
            case BACKWARD:
                this.frontLeft.setPower(-speed);
                this.frontRight.setPower(-speed);
                this.backLeft.setPower(-speed);
                this.backRight.setPower(-speed);
                break;
            case LEFT:
                this.frontLeft.setPower(-speed);
                this.frontRight.setPower(speed);
                this.backLeft.setPower(speed);
                this.backRight.setPower(-speed);
                break;
            case RIGHT:
                this.frontLeft.setPower(speed);
                this.frontRight.setPower(-speed);
                this.backLeft.setPower(-speed);
                this.backRight.setPower(speed);
                break;
            case ROTLEFT:
                this.frontLeft.setPower(-speed);
                this.frontRight.setPower(speed);
                this.backLeft.setPower(-speed);
                this.backRight.setPower(speed);
                break;
            case ROTRIGHT:
                this.frontLeft.setPower(speed);
                this.frontRight.setPower(-speed);
                this.backLeft.setPower(speed);
                this.backRight.setPower(-speed);
                break;
            default:
                throw new RuntimeException("SimpleAutonomous::move: bad direction!");
        }
        AutoUtil.safeWait(millis);
    }

    public void run(float speed) {
        try {
            this.move(Direction.FORWARD, 1500, speed);
            this.move(Direction.LEFT, 1000, speed);
        } catch (AutoUtil.OpModeInterruptedException e) {
            RobotLog.e(e.getMessage());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);

        this.frontLeft = this.hardwareMap.get(DcMotor.class, "drivefl");
        this.frontRight = this.hardwareMap.get(DcMotor.class, "drivefr");
        this.backLeft = this.hardwareMap.get(DcMotor.class, "drivebl");
        this.backRight = this.hardwareMap.get(DcMotor.class, "drivebr");

        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.waitForStart();

        this.run(.2f);

        this.frontLeft.setPower(0.0f);
        this.frontRight.setPower(0.0f);
        this.backLeft.setPower(0.0f);
        this.backRight.setPower(0.0f);
    }
}
