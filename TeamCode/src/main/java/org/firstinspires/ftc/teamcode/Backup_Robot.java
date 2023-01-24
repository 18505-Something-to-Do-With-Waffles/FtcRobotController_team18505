package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// For Controller
import com.qualcomm.robotcore.hardware.Gamepad;
// For IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
// For Vision
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;



public class Backup_Robot {

    // Instantiate Telemetry
    private Telemetry telemetry;

    // Instantiate Controller
    private Controller controller1;

    // Instantiate Lift System
    private int liftPos;
    final private int[] posEncoderVal = {0, 610, 1075, 1460};
    private LiftSystem liftSystem;

    // Instantiate Drive System
    private DriveSystem driveSystem;

    // Instantiate Grip System
    private boolean gripPos;
    private GripSystem gripSystem;

    // Instantiate IMU System
    private IMUSystem imu;

    // Instantiate DeadWheelEncoder System
    private DeadWheelEncoderSystem dwEncoder;

    // Instantiate DistanceSensor
    private DistanceSensor distanceSensor;

    // Instantiate VisionSystem
    private VisionSystem vision;



    // Initialize stuff
    public Backup_Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        //
        this.telemetry = telemetry;

        // Initialize IMU
        this.imu = new IMUSystem(hardwareMap);

        // Initialize DriveSystem
        this.driveSystem = new DriveSystem(hardwareMap);

        // Initialize LiftSystem
        this.liftPos = 0;
        this.liftSystem = new LiftSystem(hardwareMap);

        //Initialize GripSystem
        this.gripSystem = new GripSystem(hardwareMap);

        // Initialize DeadWheelEncoder System
        this.dwEncoder = new DeadWheelEncoderSystem(hardwareMap);

        // Initialize Distance Sensor
        this.distanceSensor = new DistanceSensor(hardwareMap);

        //Initialize Vision System
        this.vision = new VisionSystem(hardwareMap);
    }

    public double getHeading() {
        return this.imu.getHeading();
    }

    public void resetHeading() {
        this.imu.resetHeading();
    }

    public void teleDrive(Controller controller) {
        double speedMultiplier = 1;
        /* [TBD] adjust speedMultiplier based on lift position */
        /* [TBD] get x, y, r from controller */
        double x = 0; // [TBD] get value from controller
        double y = 0; // [TBD] get value from controller
        double r = 0; // [TBD] get value from controller
        this.driveSystem.drive(x, y, r, speedMultiplier);
    }

    public void autoDriveDist(double targetDist) {
        // [TBD] Will involve dwEncoder
        // [TBD] Define auto drive behavior in terms of x, y, and r
        // [TBD] Call driveSystem.drive() using calculated x, y, r
    }

    public void autoTurnToHeading(double targetHeading) {
        // [TBD] will involve imu, and some modulus math
        // [TBD] Define auto drive behavior in terms of x, y, and r
        // [TBD] Call driveSystem.drive() using calculated x, y, r
    }

    public void autoStrafeDist(double targetDist) {
        // [TBD] Will involve deadwheel encoders
        // [TBD] Define auto drive behavior in terms of x, y, and r
        // [TBD] Call driveSystem.drive() using calculated x, y, r
    }

    public int getLiftPos() {
        // Returns current lift position
        return this.liftPos;
    }

    public void setLiftPos(int newLiftPos) {
        /* Sets lift position to a new discrete position found
           in the posEncoderVal array. */
        int newEncoderVal = this.posEncoderVal[newLiftPos];
        this.liftSystem.setLiftEncoderPos(newEncoderVal);
        this.liftPos = newLiftPos;
    }

    public void teleSetLiftPos(Controller controller) {
        // Wraps setLiftPos with some decision logic based on current lift position and controller
        // [TBD] get command from controller
        // [TBD] get current lift position
        // [TBD] set new lift position using liftSystem.setLiftPos()
    }

    public void teleAdjustLiftPos(Controller controller) {
        // I think the desired behavior is if a certain button is held, the lift will move slowly up/down
        // [TBD] get command from controller
        // [TBD] increment/decrement lift position by some increment value using liftSystem.setLiftEncoderPos()
        // [TBD] may need to update this.liftPos, depending on if 4 represents new position
    }

    public void teleLift(Controller controller) {
        // Wrapper function which will call both teleop modes of lift operation
        this.teleSetLiftPos(controller);
        this.teleAdjustLiftPos(controller);
    }

    public void teleGripper(Controller controller) {
        // Use controller variables to call this.gripperSystem methods
        if (controller.AOnce()) {
            gripSystem.setPosition(0.02, 1);
        }
        if (controller.BOnce()) {
            gripSystem.setPosition(0.66, 0.37);
        }
        telemetry.addData("left position", gripSystem.getPosition());
    }
}

class Controller {
    // Defines controller behavior
    private Gamepad gamepad;

    /* This is kind of a neat implementation:
       https://github.com/cporter/ftc_app/blob/vv/autonomous-testing/TeamCode/src/main/java/com/suitbots/vv/Controller.java
    */

    // Per the above link, these are the int values that will increment as buttons are held down
    // private because we don't want to expose them.  We want to expose methods that use them
    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b;
    private int left_bumper, right_bumper;
    // These will simply pass the value from the gamepad object out through the Controller object
    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;
    // [TBD]


    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // This updates all of the controller object's class variables according to gamepad actions
        // this method should be called once for every teleop loop
        if (gamepad.x) { ++x; } else { x = 0; }
        if (gamepad.y) { ++y; } else { y = 0; }
        if (gamepad.a) { ++a; } else { a = 0; }
        if (gamepad.b) { ++b; } else { b = 0; }
        if (gamepad.dpad_up) { ++this.dpad_up; } else { this.dpad_up = 0; }
        if (gamepad.dpad_down) { ++this.dpad_down; } else { this.dpad_down = 0; }
        if (gamepad.dpad_left) { ++this.dpad_left; } else { this.dpad_left = 0; }
        if (gamepad.dpad_right) { ++this.dpad_right; } else { this.dpad_right = 0; }
        if (gamepad.left_bumper) { ++this.left_bumper; } else { this.left_bumper = 0; }
        if (gamepad.right_bumper) { ++this.right_bumper; } else { this.right_bumper = 0; }

        this.left_stick_x = gamepad.left_stick_x;
        this.left_stick_y = gamepad.left_stick_y;
        this.right_stick_x = gamepad.right_stick_x;
        this.right_stick_y = gamepad.right_stick_y;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;
    }

    // These methods are for "is currently pressed" (i.e. "hold" behavior)
    public boolean dpadUp() { return 0 < this.dpad_up; }
    public boolean dpadDown() { return 0 < this.dpad_down; }
    public boolean dpadLeft() { return 0 < this.dpad_left; }
    public boolean dpadRight() { return 0 < this.dpad_right; }
    public boolean X() { return 0 < this.x; }
    public boolean Y() { return 0 < this.y; }
    public boolean A() { return 0 < this.a; }
    public boolean B() { return 0 < this.b; }
    public boolean leftBumper() { return 0 < this.left_bumper; }
    public boolean rightBumper() { return 0 < this.right_bumper; }

    // These methods are for "was just pressed" (i.e. press once behavior)
    public boolean dpadUpOnce() { return 1 == this.dpad_up; }
    public boolean dpadDownOnce() { return 1 == this.dpad_down; }
    public boolean dpadLeftOnce() { return 1 == this.dpad_left; }
    public boolean dpadRightOnce() { return 1 == this.dpad_right; }
    public boolean XOnce() { return 1 == this.x; }
    public boolean YOnce() { return 1 == this.y; }
    public boolean AOnce() { return 1 == this.a; }
    public boolean BOnce() { return 1 == this.b; }
    public boolean leftBumperOnce() { return 1 == this.left_bumper; }
    public boolean rightBumperOnce() { return 1 == this.right_bumper; }

}

class DeadWheelEncoderSystem {
    // [TBD] Declare wheel circumference constants for distance calcs
    // [TBD] Instantiate all encoders

    public DeadWheelEncoderSystem(HardwareMap hardwareMap) {
        // [TBD] Initialize all encoders, class variables
    }
    // [TBD] Methods here
} // [TBD]

class DistanceSensor {
    // [TBD] Instantiate distance sensor

    public DistanceSensor(HardwareMap hardwareMap) {
        // [TBD] Initialize distance sensor
    }
    // [TBD] Methods here
} // [TBD]

class DriveSystem {
    private DcMotor rearleft;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor rearright;


    // Constructor
    public DriveSystem(HardwareMap hardwareMap) {
        this.rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        this.frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        this.frontright = hardwareMap.get(DcMotor.class, "frontright");
        this.rearright = hardwareMap.get(DcMotor.class, "rearright");
    }

    public void drive(double x, double y, double r, double speedMultiplier) {
        double movementy = y * speedMultiplier;
        double movementx = x * speedMultiplier;
        double movementr = x * speedMultiplier * 1.5;
        // [TBD] It seems like the inputs to setPower should be clipped (0 to 1)
        rearleft.setPower((movementy + movementx) - movementr);
        frontleft.setPower((movementy + (0 - movementx)) - movementr);
        frontright.setPower(((0 - movementy) + (0 - movementx)) - movementr);
        rearright.setPower(((0 - movementy) + movementx) - movementr);
    }

    public void strafe() {

    } // [TBD] Not sure if this is actually needed since this.drive can strafe
}

class GripSystem {
    // [TBD]
    private Servo left;
    private Servo right;

    //Constructor
    public GripSystem(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
    }

    // [TBD] Define getPosition method (if needed)
    public double getPosition() {
        return left.getPosition();
        // [TBD] right position is not returned - maybe that's ok
    }

    // [TBD] Define setPosition method
    public void setPosition(double leftPos, double rightPos) {
        left.setPosition(leftPos);
        right.setPosition(rightPos);
    }
}

class IMUSystem {
    private IMU imu;
    private double headingOffset;

    // Constructor
    public IMUSystem(HardwareMap hardwareMap) {
        this.imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getHeading() {
        double heading;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.DEGREES) - this.headingOffset;
        if (heading < -180) heading %= 360;
        return heading;
    }

    public void resetHeading() {
        headingOffset = 0.0; // zero offset so next call will be raw data
        headingOffset = this.getHeading();
    }
}

class LiftSystem {
    private int liftPos;
    private DcMotor lift5rear;
    private DcMotor lift4front;
    final private int top = 1480;
    final private int bottom = 0;

    // Constructor
    public LiftSystem(HardwareMap hardwareMap) {
        this.lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
        this.lift4front = hardwareMap.get(DcMotor.class, "lift4front");
        lift4front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift5rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift4front.setDirection(DcMotor.Direction.REVERSE);
        lift4front.setDirection(DcMotor.Direction.FORWARD);
        lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift4front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift5rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int[] getLiftEncoderPos() {
        int[] frontRearLiftEncoderPos = {lift4front.getCurrentPosition(), lift5rear.getCurrentPosition()};
        return frontRearLiftEncoderPos;
    }

    public void setLiftEncoderPos(int newLiftEncoderPos) {
        if (newLiftEncoderPos >= bottom && newLiftEncoderPos <= top) {
            lift4front.setTargetPosition(newLiftEncoderPos);
            lift5rear.setTargetPosition(newLiftEncoderPos);
        }
    }
}

class VisionSystem {
    // [TBD] TFOD, Vuforia, and Webcam declarations
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    public VisionSystem(HardwareMap hardwareMap) {
        // [TBD] TFOD, Vuforia, and Webcam initialization
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
    }
    // [TBD] Methods here
    /*Could create readCone() here, or a more generalized version getHighestConfLabel()
    which could take an array of confidence thresholds as optional parameter.  That would let
    you reuse this class/method in future years without changing it.*/

} // [TBD]


