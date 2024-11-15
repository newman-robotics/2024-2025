package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "SimpleAutonomous")
public class SimpleAutonomous extends LinearOpMode {
    public enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        ROTLEFT,
        ROTRIGHT
    }

    AutoUtil.Hardware hardware;

    /**
     * Moves the robot for the specified time.
     * @param direction The direction to move the robot in.
     * @param millis The time to move the robot for, in milliseconds.
     * @param speed The speed at which to move the robot.
     * @throws AutoUtil.OpModeInterruptedException Throws if the OpMode is interrupted or deactivates.
     * **/
    public void move(Direction direction, long millis, double speed) throws AutoUtil.OpModeInterruptedException {
        switch (direction) {
            case FORWARD:
                hardware.setDrivetrainPowers(speed, speed, speed, speed);
                break;
            case BACKWARD:
                hardware.setDrivetrainPowers(-speed, -speed, -speed, -speed);
                break;
            case LEFT:
                hardware.setDrivetrainPowers(-speed, speed, speed, -speed);
                break;
            case RIGHT:
                hardware.setDrivetrainPowers(speed, -speed, -speed, speed);
                break;
            case ROTLEFT:
                hardware.setDrivetrainPowers(-speed, speed, -speed, speed);
                break;
            case ROTRIGHT:
                hardware.setDrivetrainPowers(speed, -speed, speed, -speed);
                break;
            default:
                throw new RuntimeException("SimpleAutonomous::move: bad direction!");
        }
        AutoUtil.safeWait(millis);
    }

    public void run() {
        try {
            this.move(Direction.RIGHT, GlobalConstants.MOTION_MILLIS_TILE * 2, GlobalConstants.MOTION_SPEED);
        } catch (AutoUtil.OpModeInterruptedException e) {
            RobotLog.e(e.getMessage());
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);
        this.hardware = AutoUtil.Hardware.init(this.hardwareMap);

        this.waitForStart();

        this.run();

        this.hardware.zeroOut();
    }
}
