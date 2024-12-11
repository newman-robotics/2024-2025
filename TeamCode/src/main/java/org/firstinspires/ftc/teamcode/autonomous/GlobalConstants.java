package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.hardware.Gamepad;

/* Configuration:
Expansion hub (Servos): port 5 = claw
Expansion hub (Servos): port 3 = intake
Expansion hub (Servos): port 2 = wrist (Continuous Rotation)

Expansion hub (Motors): port 0 = actuator
Expansion hub (Motors): port 1 = elbow
Expansion hub (Motors): port 2 = elbow2
Expansion hub (Motors): port 3 = elevation
* */

// making a class with all the control parts and there names n stuff
public class GlobalConstants {
    //These are the buttons on the controllers
    public enum GamepadInput {
        // these control the joysticks
        LEFT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_X,
        RIGHT_STICK_Y,

        //These are the buttons
        BUTTON_A,
        BUTTON_B,
        BUTTON_X,
        BUTTON_Y,

        //These are for the Dpads
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,

        //these are for the triggers
        LEFT_TRIGGER,
        RIGHT_TRIGGER,

        //these are for the bumpers, or the small buttons next to the triggers
        LEFT_BUMPER,
        RIGHT_BUMPER,

        //On each joystick if you push down it triggers these buttons
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
    }

    //this sets the motors from the robot configuration PAWPAW to variables with similar names
    //these are all the wheel motors
    public static final String FRONT_LEFT_MOTOR_NAME = "drivefl";
    public static final String FRONT_RIGHT_MOTOR_NAME = "drivefr";
    public static final String BACK_LEFT_MOTOR_NAME = "drivebl";
    public static final String BACK_RIGHT_MOTOR_NAME = "drivebr";

    //these are motors that are used in the arm, the actuator raises the arm up and down
    //the elbow is like an elbow attached at the end of the actuator to control where the claw goes
    public static final String ARM_VERTICAL_MOTOR_NAME = "actuator";
    @Deprecated
    public static final String OLD_ARM_ELBOW_MOTOR_NAME = "elbow";

    //these are servos that control the claw, the wrist is like a wrist and controls the angle of the claw
    //the intake is a system that will grab the pieces and store them
    @Deprecated
    public static final String OLD_CLAW_WRIST_MOTOR_NAME = "wrist";
    @Deprecated
    public static final String OLD_CLAW_INTAKE_MOTOR_NAME = "intake";

    public static final String ARM_ELBOW_MOTOR_NAME = "elbow";
    public static final String ARM_VERTICAL_SERVO_MOTOR_NAME = "elevation";
    public static final String ARM_LINEAR_SLIDE_NAME = "elevation";
    public static final String CLAW_MOTOR_NAME = "claw";

    public static final String CLAW2_FIRST_MOTOR_NAME = "wrist";
    public static final String CLAW2_SECOND_MOTOR_NAME = "intake";

    //This might be used in a third version of the arm
    public static final String ARM3_TOP_SERVO_TIGHTENER_NAME = "CRServo_tightener";

    public static final String ARM3_BOTTOM_SERVO_CLAW_NAME = "Servo_claw";

    public static final String ARM3_BASE_NAME = "Base_motor";


    //this isn't a motor but controls the camera
    public static final String WEBCAM_NAME = "webcam";

    //this creates a value that the triggers must reach in order to trigger a motor
    public static final double GAMEPAD_THRESHOLD = 0.7;

    //the slow factor is multiplied by all the motor values when the slow factor is activated, slowing the robot
    public static final double SLOW_FACTOR = 0.25;

    //creates an input and sets that input to one of the controller button
    //this is used in a switch case that determines the power set to a motor
    //this is mostly unnecessary Owen nonsense
    public static final GamepadInput AXIAL = GamepadInput.LEFT_STICK_X;
    public static final GamepadInput LATERAL = GamepadInput.LEFT_STICK_Y;
    public static final GamepadInput YAW = GamepadInput.RIGHT_STICK_X;
    public static final GamepadInput SLOW = GamepadInput.BUTTON_A;
    public static final GamepadInput XVARTESTNEWCODE = GamepadInput.BUTTON_Y;

    public static final GamepadInput ARM_CLAW_INPUT = GamepadInput.RIGHT_STICK_Y;
    @Deprecated
    public static final GamepadInput OLD_ARM_ELEVATION_MODIFIER = GamepadInput.LEFT_BUMPER;
    @Deprecated
    public static final GamepadInput OLD_ARM_ELBOW_MODIFIER = GamepadInput.RIGHT_BUMPER;
    @Deprecated
    public static final GamepadInput OLD_CLAW_INTAKE_MODIFIER = GamepadInput.LEFT_TRIGGER;
    @Deprecated
    public static final GamepadInput OLD_CLAW_WRIST_MODIFIER = GamepadInput.RIGHT_TRIGGER;

    public static final GamepadInput ARM_LOWER_MODIFIER = GamepadInput.LEFT_TRIGGER;
    public static final GamepadInput ARM_UPPER_MODIFIER = GamepadInput.RIGHT_TRIGGER;
    public static final GamepadInput ARM_ELBOW_MODIFIER = GamepadInput.LEFT_BUMPER;
    public static final GamepadInput CLAW_MODIFIER = GamepadInput.RIGHT_BUMPER;


    //
    public static final double ARM_ELBOW_TICK_MODIFIER = 100.0;
    public static final double ARM_ELBOW_SPEED = 0.8;

    public static final double CLAW_WRIST_POSITION_MODIFIER = 0.01;

    /**
     * @deprecated The claw is installed, and is likely not coming off anytime soon.
     * **/
    @Deprecated
    public static final boolean CLAW_IS_INSTALLED = true;

    //Forward
    //Sideways
    //Backwards

    //Motion_millis controls the time that the backup autonomous code makes a motor go for
    //so everytime backup autonomous runs a movement (backwards, forwards) it last for 1000 milliseconds
    public static final long MOTION_MILLIS_TILE_FULL_SPEED = 1000;

    //this is how fast the backup autonomous motors go when called
    public static final double MOTION_SPEED = 1.0;


    //this does what it says and divides the time by the speed and gets a value
    //if 1000 milliseconds and a motion speed of 1 moved the robot one foot,
    //then changing the speed to two would cut the time in half causing it to still go one foot in total
    //however currently we don't have a total for how much distance is covered when the robot goes at a speed of one for 1000 millis
    //this currently controls the time for some reason even though it shouldn't because we haven't calibrated it yet
    //if some does calibrate this please UPDATE THIS COMMENT


    public static final long MOTION_MILLIS_TILE = (long)Math.floor(MOTION_MILLIS_TILE_FULL_SPEED / MOTION_SPEED);

    public static final int MAX_CAMERA_PROCESSING_THREADS = 10;

    public static final int CAMERA_X_SIZE = 640;
    public static final int CAMERA_Y_SIZE = 480;

    public static final String ODOMETRY_NAME = "odo";
    //millis
    public static final double ODOMETRY_X_OFFSET = DistanceUnit.mmPerInch * 3.25;
    public static final double ODOMETRY_Y_OFFSET = DistanceUnit.mmPerInch * 6;

    public static final GamepadInput INPUT_ACTUATOR_UP = GamepadInput.LEFT_TRIGGER;
    public static final GamepadInput INPUT_ACTUATOR_DOWN = GamepadInput.RIGHT_TRIGGER;
    public static final GamepadInput INPUT_ELBOW_UP = GamepadInput.LEFT_BUMPER;
    public static final GamepadInput INPUT_ELBOW_DOWN = GamepadInput.RIGHT_BUMPER;
    public static final GamepadInput INPUT_LINEAR_SLIDE_UP = GamepadInput.DPAD_UP;
    public static final GamepadInput INPUT_LINEAR_SLIDE_DOWN = GamepadInput.DPAD_DOWN;
    public static final GamepadInput INPUT_CLAW_OPEN = GamepadInput.BUTTON_A;
}
