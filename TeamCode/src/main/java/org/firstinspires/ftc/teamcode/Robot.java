package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// For Controller
import com.qualcomm.robotcore.hardware.Gamepad;
// For IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// For Motors
import com.qualcomm.robotcore.hardware.DcMotor;
// For Servos
import com.qualcomm.robotcore.hardware.Servo;
// For Vision
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;

import java.util.List;
import java.lang.*;


public class Robot {

    // Instantiate Telemetry
    private Telemetry telemetry;

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
    private String modelFile = "model_20221228_153128.tflite";



    // Initialize stuff
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        //
        this.telemetry = telemetry;

        // Initialize IMU
        this.imu = new IMUSystem(hardwareMap);

        // Reset IMU heading - localized to playing field
        imu.resetHeading();

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
        String[] visionElements = {"blue", "green", "red"};
        this.vision = new VisionSystem(hardwareMap, visionElements, 1.6, 1, modelFile);
    }

    public double getHeading() {
        return this.imu.getHeading();
    }

    public void resetHeading() {
        this.imu.resetHeading();
    }

    public void teleDrive(Controller controller) {
        double speedMultiplier;
        if (this.getLiftPos() == 0) {
            speedMultiplier = 0.45;
        } else {
            speedMultiplier = 0.3;
        }

        double x = controller.left_stick_x * speedMultiplier;
        double y = controller.left_stick_y * speedMultiplier;
        double r = controller.right_stick_x * speedMultiplier;
        this.driveSystem.drive(x, y, r);
    }

    public void autoDriveDist(double targetDist, double speed) {
        double driveEncoderDistance;
        driveEncoderDistance = 45.2849 * targetDist;

        driveSystem.resetEncoder();
        while (Math.abs(driveEncoderDistance - driveSystem.getEncoder()) > 20){
            if (targetDist < 0) {driveSystem.drive(0,speed, 0);}
            else{driveSystem.drive(0,-speed, 0);}
        }
        driveSystem.drive(0,0, 0);
    }

    public void autoTurnToHeading(double targetHeading, double speed) {
        targetHeading = -targetHeading;
        double turnspeed;
//        double turnAngle = (targetHeading - this.getHeading() + 180) % 360 - 180;
        double turnAngle = (this.getHeading() -targetHeading + 180) % 360 - 180;

        while (Math.abs(turnAngle)>1){
//            turnAngle = targetHeading - this.getHeading();
            turnAngle = this.getHeading() - targetHeading;
            turnAngle = (turnAngle + 180) % 360 - 180;
            turnspeed = speed*Math.max(Math.min(Math.abs(turnAngle)/20,1), 0.1);
            if (turnAngle < 0){
                turnspeed = -turnspeed;
            }

            driveSystem.drive(0, 0, turnspeed);
        }
        driveSystem.drive(0,0, 0);
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
        boolean up = controller.dpadUpOnce();
        boolean down = controller.dpadDownOnce();
        switch (this.getLiftPos()) {
            case 0:
                if (up){
                    this.setLiftPos(1);
                }
            case 1:
                if (up){
                    this.setLiftPos(2);
                }
                else if (down){
                    this.setLiftPos(0);
                }
            case 2:
                if (up){
                    this.setLiftPos(3);
                }
                else if (down){
                    this.setLiftPos(0);
                }
            case 3:
                if (down){
                    this.setLiftPos(0);
                }
            case 4:
                if (down){
                    this.setLiftPos(0);
                }
                else if (up){
                    int j = 0;
                    while((posEncoderVal[j] < this.liftSystem.getLiftEncoderPos()[0]) && (j < 3)){
                        ++j;
                    }
                    this.setLiftPos(j);
                    this.liftPos = j;
                }
        }
    }

    public void teleAdjustLiftPos(Controller controller) {
        boolean up = controller.dpadRight();
        boolean down = controller.dpadLeft();
        if (up) {
            liftSystem.setLiftEncoderPos(liftSystem.getLiftEncoderPos()[0]+20);
            this.liftPos = 4;
        }
        else if (down){
            liftSystem.setLiftEncoderPos(liftSystem.getLiftEncoderPos()[0]-20);
            this.liftPos = 4;
        }
        // I think the desired behavior is if a certain button is held, the lift will move slowly up/down
        // [TBD] get command from controller
        // [TBD] increment/decrement lift position by some increment value using liftSystem.setLiftEncoderPos()
        // [TBD] may need to update this.liftPos, depending on if 4 represents new position
    }

    public void teleLift(Controller controller) {
        // Wrapper function which will call both teleop modes of lift operation
        this.teleSetLiftPos(controller);
        this.teleAdjustLiftPos(controller);
        this.liftSystem.lowPowerMode();
    }

    public void teleGripper(Controller controller) {
        // Use controller variables to call this.gripperSystem methods
        if (controller.AOnce()) {
            gripper(true);
        }
        if (controller.BOnce()) {
            gripper(false);
        }
    }

    public void gripper(Boolean close){
        if (close){
            gripSystem.setPosition(0.02, 1);
        }
        else {
            gripSystem.setPosition(0.66, 0.37);
        }
    }

    public int readCone(){
        return this.vision.readCone();
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

    public void drive(double x, double y, double r) {
        double movementy = y;
        double movementx = x;
        double movementr = r * 1.5;
        // [TBD] It seems like the inputs to setPower should be clipped (0 to 1)
        rearleft.setPower((movementy + movementx) - movementr);
        frontleft.setPower((movementy + (0 - movementx)) - movementr);
        frontright.setPower(((0 - movementy) + (0 - movementx)) - movementr);
        rearright.setPower(((0 - movementy) + movementx) - movementr);
    }

    public void resetEncoder(){
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public double getEncoder(){
        return rearright.getCurrentPosition();
    }
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
    final private double power = 0.5;
    private int targetPosition;

    // Constructor
    public LiftSystem(HardwareMap hardwareMap) {
        this.targetPosition = 0;
        this.lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
        this.lift4front = hardwareMap.get(DcMotor.class, "lift4front");
        lift4front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift5rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift4front.setDirection(DcMotor.Direction.REVERSE);
        lift5rear.setDirection(DcMotor.Direction.FORWARD);
        lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int[] getLiftEncoderPos() {
        int[] frontRearLiftEncoderPos = {lift4front.getCurrentPosition(), lift5rear.getCurrentPosition()};
        return frontRearLiftEncoderPos;
    }

    public void setLiftEncoderPos(int newLiftEncoderPos) {
        if (newLiftEncoderPos >= bottom && newLiftEncoderPos <= top) {
            this.targetPosition = newLiftEncoderPos;
            lift4front.setPower(power);
            lift5rear.setPower(power);
            lift4front.setTargetPosition(newLiftEncoderPos);
            lift5rear.setTargetPosition(newLiftEncoderPos);
            lift4front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift5rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void lowPowerMode(){
        if (this.getLiftEncoderPos()[0] < 10 && this.targetPosition == 0){
            lift4front.setPower(0);
            lift5rear.setPower(0);
            lift4front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift5rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

class VisionSystem{
    // [TBD] TFOD, Vuforia, and Webcam declarations
    private VuforiaCurrentGame vuforiaPOWERPLAY;
    private Tfod tfod;

    public VisionSystem(HardwareMap hardwareMap, String[] elements, double zoom, double aspectRatio, String modelFile) {
        vuforiaPOWERPLAY = new VuforiaCurrentGame();
        tfod = new Tfod();
        vuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        tfod.useModelFromFile(modelFile, JavaUtil.createListWith(elements), true, true, 320);
        tfod.initialize(vuforiaPOWERPLAY, (float) 0.55, true, true);
        tfod.setClippingMargins(0, 0, 0, 0);
        tfod.activate();
        // Enable following block to zoom in on target.
        tfod.setZoom(zoom, aspectRatio);
    }
    public int readCone(){
        // [TBD] separate POWERPLAY details from vision system and put in robot
        int cameraReads = 2;
        List<Recognition> recognitions;
        Recognition recognition;
        double highestConfidence = 0;
        long time = System.currentTimeMillis();
        final double secondsToSearch = 2;
        while (System.currentTimeMillis() - time < secondsToSearch*1000) {
            recognitions = tfod.getRecognitions();
            for (Recognition recognition_item : recognitions) {
                recognition = recognition_item;
                if (recognition_item.getLabel().equals("green") && recognition_item.getConfidence() > .85){
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 3;
                        highestConfidence = recognition.getConfidence();
                    }
                }
                else if(recognition_item.getLabel().equals("red")) {
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 1;
                        highestConfidence = recognition.getConfidence();
                    }
                }
                else if(recognition_item.getLabel().equals("blue")) {
                    if (recognition_item.getConfidence() > highestConfidence) {
                        cameraReads = 2;
                        highestConfidence = recognition.getConfidence();
                    }
                }
            }
        }
        return cameraReads;

    }
    // [TBD] Methods here
    /*Could create readCone() here, or a more generalized version getHighestConfLabel()
    which could take an array of confidence thresholds as optional parameter.  That would let
    you reuse this class/method in future years without changing it.*/

} // [TBD]


