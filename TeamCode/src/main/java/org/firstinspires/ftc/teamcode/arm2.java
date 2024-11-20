package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;

public class arm2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo rope_tightener = null;
    private CRServo upy_downly = null;
    private CRServo grippy = null;








    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);
        boolean running = true;

        this.waitForStart();


        while (this.opModeIsActive()) {
            boolean want_to_lift = gamepad1.right_bumper;
            boolean want_to_drop_down = gamepad1.left_bumper;


            boolean hand_closes = gamepad1.b;

            float fason = gamepad1.left_trigger;
            float fason2 = gamepad1.right_trigger;


            //motors setting need to name on the drive hub
            rope_tightener = hardwareMap.get(CRServo.class, "rope_tightener");
            upy_downly = hardwareMap.get(CRServo.class, "Rope_Upy_downy");
            grippy = hardwareMap.get(CRServo.class, "Rope_arm)");

            // setter we will see what happens
            upy_downly.setDirection(DcMotorSimple.Direction.FORWARD);

            grippy.setDirection(DcMotorSimple.Direction.FORWARD);


            //inputs and stuff
            if (want_to_lift) {
                upy_downly.setPower(10);

            }

            if (want_to_drop_down) {

                upy_downly.setPower(-10);
            }

            if (hand_closes) {
                grippy.setPower(-10);
            }
            if (!hand_closes) {
                grippy.setPower(10);
            }

            if (fason > .5f) {
                rope_tightener.setPower(10);
            }

            if (fason2 > .5f) {
                rope_tightener.setPower(-10);
            }
        }









    }
}