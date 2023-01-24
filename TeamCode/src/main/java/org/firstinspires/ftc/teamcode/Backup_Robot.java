package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.HardwareMap;
// For Controller
import com.qualcomm.robotcore.hardware.Gamepad;
// For IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;


class Controller {
    // Defines controller behavior
    private Gamepad gamepad;
    // [TBD]
    /* This is kind of a neat implementation:
       https://github.com/cporter/ftc_app/blob/vv/autonomous-testing/TeamCode/src/main/java/com/suitbots/vv/Controller.java
    */

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // This updates all of the controller object's class variables according to gamepad actions
        // [TBD]
    }

    public boolean once(int button) {
        // Rports true only once per button press
        return 1 == button;
    }

    public boolean hold(int button) {
        // Reports true continuously if button is held down
        return button > 0;
    }
}


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
        rearleft.setPower((movementy + movementx) - movementr);
        frontleft.setPower((movementy + (0 - movementx)) - movementr);
        frontright.setPower(((0 - movementy) + (0 - movementx)) - movementr);
        rearright.setPower(((0 - movementy) + movementx) - movementr);
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

class GripSystem {
    // [TBD]
    //Constructor
    public GripSystem(HardwareMap hardwareMap) {
        // [TBD]
    }
    // [TBD] Define getPosition method

    // [TBD] Define setPosition method

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

class DeadWheelEncoderSystem {
    // [TBD] Declare wheel circumference constants for distance calcs
    // [TBD] Instantiate all encoders

    public DeadWheelEncoderSystem(HardwareMap hardwareMap) {
        // [TBD] Initialize all encoders, class variables
    }
    // [TBD] Methods here
}

class DistanceSensor {
    // [TBD] Instantiate distance sensor

    public DistanceSensor(HardwareMap hardwareMap) {
        // [TBD] Initialize distance sensor
    }
    // [TBD] Methods here
}

class VisionSystem {
    // [TBD] TFOD, Vuforia, and Webcam declarations

    public VisionSystem(HardwareMap hardwareMap) {
        // [TBD] TFOD, Vuforia, and Webcam initialization
    }
    // [TBD] Methods here
}

public class Backup_Robot {

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

        // Initialize IMU
        imu = new IMUSystem(hardwareMap);

        // Initialize DriveSystem
        driveSystem = new DriveSystem(hardwareMap);

        // Initialize LiftSystem
        this.liftPos = 0;
        liftSystem = new LiftSystem(hardwareMap);

        //Initialize GripSystem
        gripSystem = new GripSystem(hardwareMap);

        // Initialize DeadWheelEncoder System
        dwEncoder = new DeadWheelEncoderSystem(hardwareMap);

        // Initialize Distance Sensor
        distanceSensor = new DistanceSensor(hardwareMap);

        //Initialize Vision System
        vision = new VisionSystem(hardwareMap);
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
    }
}
