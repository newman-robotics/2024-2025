//importing important things
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="New_Code_Juju")
public class NotCopiedDrive extends LinearOpMode {

    //creates runtime for when the robot is running
    private final ElapsedTime runtime = new ElapsedTime();


    //NO PASTING
    @Override
    public void runOpMode() throws InterruptedException {

        //creating variables and assigning them to the motors
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "drivefl");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "drivebl");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "drivefr");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "drivebr");

        //omni wheels can be weird so some of the motors have to go in reverse
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //these values are displayed on the driver hub
        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {

            //creating variables that trigger when a button on the control is pressed
            double max;
            double axial = -gamepad1.left_stick_y;  //pushing stick forward gives negative value so we have to make it negetive
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean slow = gamepad1.a;

            //this assigns a variable to each combination of movement for the robot
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            //creating a max amount of power to prevent the motors from reaching lightspeed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower = leftFrontPower / max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            //Early we assigned gamepad1.a to slow, so now if you click it,
            // it will slow the robot down by a fourth. If not it will drive as usual
            if (slow) {
                leftFrontDrive.setPower(leftFrontPower / 4);
                rightFrontDrive.setPower(rightFrontPower / 4);
                leftBackDrive.setPower(leftBackPower / 4);
                rightBackDrive.setPower(-rightBackPower / 4);
            }

            else {
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(-rightBackPower);
            }

            //telemetry is the data shown on teh drive hub
            //in most cases it doesn't do much but can be useful for testing code
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //this one says slow if slow mode is activated
            telemetry.addData("slow", slow);
            telemetry.addData(">", "Press Stop to end test." );
            idle();
            telemetry.addData(">", "Done");
            telemetry.update();

        }
    }
}
