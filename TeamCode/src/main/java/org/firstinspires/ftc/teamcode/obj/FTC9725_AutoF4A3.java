package org.firstinspires.ftc.teamcode.obj;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="FTC9725 Test: Auto Mode F4A3", group="Test Robot")

public class FTC9725_AutoF4A3 extends LinearOpMode {

    /* Declare OpMode members. */
    // Robot name is robot
    Huynh_BotHardware_test  robot = new Huynh_BotHardware_test(this); // use hardware file


    public void safeWait(long millis) {
        long deadline = System.currentTimeMillis() + millis;
        while (System.currentTimeMillis() != deadline)
            if (!(this.opModeInInit() || this.opModeIsActive()))
                throw new RuntimeException("Interrupted!");
    }

    public double DistanceToTime (double speed, double distance){
        double time = distance/speed; //time in seconds, speed in cm/s
        // axial: 0.2speed = 9.45in/s, 0.5speed = 36in/s
        // lateral: 0.5speed = 22in/s
        return time;
    }


    static final double     FORWARD_SPEED = 0.2;
    // 0.2 speed = 24 cm/s
    //
    static final double     TURN_SPEED    = 0.2;
    // 0.2 turn speed = 2.55 sec to turn 90º
    // 0.2 turn speed = 0.917 sec to turn 90º
    //idk why it isnt linear to 0.2 but wtv
    static final double     SIDE_SPEED    = 0.2;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        // The init() method of the hardware file does it all here
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        //5.5 in/second @ 0.2 speed
        robot.driveRobot(0,0,0);

        robot.wrist.setPosition(0);
        safeWait(500);

        robot.driveRobot(0.5,0,0);
        safeWait(545);
        robot.driveRobot(0,0,0);
        safeWait(300);
        //moves robot out

        robot.driveRobot(0,0,-0.5);
        safeWait(905);
        robot.driveRobot(0,0,0);
        safeWait(300);
        //turns robot 90º

        robot.driveRobot(0.5,0,0);
        safeWait(1364);
        robot.driveRobot(0,0,0);
        safeWait(300);
        //moves it closer to basket 2.5ft 30in

        robot.driveRobot(0,0,-0.5);
        safeWait(452);
        robot.driveRobot(0,0,0);
        safeWait(300);
        //turns it 45º

        robot.elbow.setPower(0.8);
        robot.elbow.setTargetPosition(-482); //change to desired point
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        safeWait(3000);
        //raises elbow

        robot.actuator.setPower(1);
        safeWait(5700);
        robot.actuator.setPower(0);
        safeWait(300);
        //raises actuator

        robot.driveRobot(0.5,0,0);
        safeWait(771);
        robot.driveRobot(0,0,0);
        //final basket move

        robot.wrist.setPosition(1);
        safeWait(1000);
        //flexes wrist up

        robot.intake.setPower(0.6);
        safeWait(3000);
        robot.intake.setPower(0);
        //ejects sample

        robot.elbow.setTargetPosition(-82); //change to desired point
        robot.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        safeWait(1000);
        //eases elbow down

        // Step 4:  Stop
        robot.driveRobot(0, 0, 0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
