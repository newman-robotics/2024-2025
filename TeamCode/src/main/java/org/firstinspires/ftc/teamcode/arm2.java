package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

@TeleOp(name = "arm2")
public class arm2 extends LinearOpMode {


    private CRServo rope_tightener = null;

    private CRServo rope_tightener2 = null;
    private CRServo upy_downly = null;

    private CRServo grippy = null;




    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);

        //Servo 0:  rope_tightener   == intake
        //Servo 1:  rope_tightener2  == wrist
        //Servo 2:` upy_downly       == arm_p1
        //Servo 3:  grippy           == Rope_arm
        rope_tightener = hardwareMap.get(CRServo.class, GlobalConstants.CLAW2_FIRST_MOTOR_NAME);
        rope_tightener2 = hardwareMap.get(CRServo.class, GlobalConstants.CLAW2_SECOND_MOTOR_NAME);
        upy_downly = hardwareMap.get(CRServo.class, "arm_p1");
        grippy = hardwareMap.get(CRServo.class, "Rope_arm)");


        this.waitForStart();



        while (this.opModeIsActive()) {
            boolean want_to_lift = gamepad1.right_bumper;
            boolean want_to_drop_down = gamepad1.left_bumper;


            boolean hand_closes = gamepad1.b;

            float fason = gamepad1.left_trigger;
            float fason2 = gamepad1.right_trigger;

            double input_detector_for_upydowny = AutoUtil.ternaryXOR(
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_TRIGGER),
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_TRIGGER)
            );
            upy_downly.setPower(input_detector_for_upydowny);

            double input_detector_for_grippy = AutoUtil.ternaryXOR(
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.BUTTON_A),
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.BUTTON_B)
            );
            grippy.setPower(input_detector_for_grippy);

            double input_detector_for_rope_tightener = AutoUtil.ternaryXOR(
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_BUMPER),
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_BUMPER)
            );
            rope_tightener.setPower(input_detector_for_rope_tightener);
            rope_tightener2.setPower(input_detector_for_rope_tightener);
        }









    }
}