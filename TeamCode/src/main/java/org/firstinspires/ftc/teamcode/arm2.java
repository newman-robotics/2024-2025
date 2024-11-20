package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;


public class arm2 extends LinearOpMode {


    private CRServo rope_tightener = null;
    private CRServo upy_downly = null;
    private CRServo grippy = null;








    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);


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

            rope_tightener.setDirection(DcMotorSimple.Direction.FORWARD);


            //inputs and stuff
            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_TRIGGER)) {
                upy_downly.setPower(.5);

            }

            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_TRIGGER)) {

                upy_downly.setPower(-.5);
            }

            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_BUMPER)) {
                grippy.setPower(-.5);
            }
            if (!AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_BUMPER)){
                grippy.setPower(.5);
            }

            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_BUMPER)){
                rope_tightener.setPower(.5);
            }

            if (!AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_BUMPER)){
                rope_tightener.setPower(-.5);
            }
        }









    }
}