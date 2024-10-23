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

    public static class OpModeInterruptedException extends Exception {
        public OpModeInterruptedException() {
            super("OpMode interrupted during sleep!");
        }

        public OpModeInterruptedException(String msg) {
            super("OpMode interrupted during sleep!\n" + msg);
        }
    }

    DcMotor frontLeft, frontRight, backLeft, backRight;

    /**
     * Tries to wait for the specified time.
     * @param millis The time to wait for, in milliseconds.
     * @throws OpModeInterruptedException Throws if the OpMode is interrupted or deactivates.
     * **/
    public void tryWait(long millis) throws OpModeInterruptedException {
        long targetTime = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() != targetTime) if (!this.opModeIsActive()) throw new OpModeInterruptedException("SimpleAutonomous::tryWait");
    }

    /**
     * Moves the robot for the specified time.
     * @param direction The direction to move the robot in.
     * @param millis The time to move the robot for, in milliseconds.
     * @param speed The speed at which to move the robot.
     * @throws OpModeInterruptedException Throws if the OpMode is interrupted or deactivates.
     * **/
    public void move(Direction direction, long millis, float speed) throws OpModeInterruptedException {
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
        this.tryWait(millis);
    }

    public void run(float speed) {
        try {
            this.move(Direction.FORWARD, 1500, speed);
            this.move(Direction.LEFT, 1000, speed);
        } catch (OpModeInterruptedException e) {
            RobotLog.e(e.getMessage());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.frontLeft = this.hardwareMap.get(DcMotor.class, "drivefl");
        this.frontRight = this.hardwareMap.get(DcMotor.class, "drivefr");
        this.backLeft = this.hardwareMap.get(DcMotor.class, "drivebl");
        this.backRight = this.hardwareMap.get(DcMotor.class, "drivebr");

        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.waitForStart();

        this.run(.2f);

        this.frontLeft.setPower(0.0f);
        this.frontRight.setPower(0.0f);
        this.backLeft.setPower(0.0f);
        this.backRight.setPower(0.0f);
    }
}
