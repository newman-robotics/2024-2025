package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="SimpleAutonomous")
public class SimpleAutonomous extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    /**
     * Tries to wait for the specified time.
     * @param millis The time to wait for, in milliseconds.
     * @return Whether the program was interrupted.
     * **/
    private boolean tryWait(long millis) {
        long targetTime = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() != targetTime) if (!this.opModeIsActive()) return true;
        return false;
    }

    private void run() {
        this.frontLeft.setPower(1.0f);
        this.frontRight.setPower(1.0f);
        this.backLeft.setPower(1.0f);
        this.backRight.setPower(1.0f);

        if (tryWait(1500)) return;

        this.frontLeft.setPower(1.0f);
        this.frontRight.setPower(-1.0f);
        this.backLeft.setPower(-1.0f);
        this.backRight.setPower(1.0f);

        tryWait(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.frontLeft = this.hardwareMap.get(DcMotor.class, "drivefl");
        this.frontRight = this.hardwareMap.get(DcMotor.class, "drivefr");
        this.backLeft = this.hardwareMap.get(DcMotor.class, "drivebl");
        this.backRight = this.hardwareMap.get(DcMotor.class, "drivebr");

        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.waitForStart();

        this.run();

        this.frontLeft.setPower(0.0f);
        this.frontRight.setPower(0.0f);
        this.backLeft.setPower(0.0f);
        this.backRight.setPower(0.0f);
    }
}
