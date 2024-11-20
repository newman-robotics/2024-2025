package org.firstinspires.ftc.teamcode.autonomous;

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
    public static final String ARM_ELBOW_MOTOR_NAME = "elbow";

    //these are servos that control the claw, the wrist is like a wrist and controls the angle of the claw
    //the intake is a system that will grab the pieces and store them
    public static final String CLAW_WRIST_MOTOR_NAME = "wrist";
    public static final String CLAW_INTAKE_MOTOR_NAME = "intake";

    //this has no use currently but if we make a new arm UPDATE THIS COMMENT
    public static final String NEW_ARM_MAYBE_NAME = "";

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

    public static final GamepadInput ARM_CLAW_INPUT = GamepadInput.RIGHT_STICK_Y;
    public static final GamepadInput ARM_ELEVATION_MODIFIER = GamepadInput.LEFT_BUMPER;
    public static final GamepadInput ARM_ELBOW_MODIFIER = GamepadInput.RIGHT_BUMPER;
    public static final GamepadInput CLAW_INTAKE_MODIFIER = GamepadInput.LEFT_TRIGGER;
    public static final GamepadInput CLAW_WRIST_MODIFIER = GamepadInput.RIGHT_TRIGGER;


    //
    public static final double ARM_ELBOW_TICK_MODIFIER = 100.0;
    public static final double ARM_ELBOW_SPEED = 0.8;

    public static final double CLAW_WRIST_POSITION_MODIFIER = 0.01;

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
}
