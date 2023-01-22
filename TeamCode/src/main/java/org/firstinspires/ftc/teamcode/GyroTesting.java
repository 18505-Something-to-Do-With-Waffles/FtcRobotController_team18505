package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Disabled
@Autonomous(name = "GyroTesting", preselectTeleOp = "Main2022")
public class GyroTesting extends LinearOpMode{

  /* Declare OpMode members. */
  private DcMotor            frontleft   = null;
  private DcMotor            frontright  = null;
  private DcMotor            rearleft = null;
  private DcMotor            rearright = null;
  private IMU                imu;   // Control/Expansion Hub IMU
  private Servo              left;
  private Servo              right;
  private DcMotor            lift4front;
  private DcMotor            lift5rear;
  private VuforiaCurrentGame vuforiaPOWERPLAY;
  private Tfod               tfod;

  private double          robotHeading  = 0;
  private double          headingOffset = 0;
  private double          headingError  = 0;

  // These variable are declared here (as class members) so they can be updated in various methods,
  // but still be displayed by sendTelemetry()
  private double  targetHeading = 0;
  private double  driveSpeed    = 0;
  private double  turnSpeed     = 0;
  private double  frontleftSpeed     = 0;
  private double  frontrightSpeed    = 0;
  private double  rearleftSpeed      = 0;
  private double  rearrightSpeed     = 0;
  private int     frontleftTarget    = 0;
  private int     frontrightTarget   = 0;
  private int     rearleftTarget    = 0;
  private int     rearrightTarget   = 0;

  // Calculate the COUNTS_PER_INCH for your specific drive train.
  // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
  // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
  // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
  // This is gearing DOWN for less speed and more torque.
  // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
  static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
  static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
  static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
  static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
          (WHEEL_DIAMETER_INCHES * 3.1415);

  // These constants define the desired driving/control characteristics
  // They can/should be tweaked to suit the specific robot drive train.
  static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
  static final double     TURN_SPEED              = 0.2;    // Max Turn speed to limit turn rate
  static final double     HEADING_THRESHOLD       = 10;     // How close must the heading get to the target before moving to next step.
  // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
  // Define the Proportional control coefficient (or GAIN) for "heading control".
  // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
  // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
  // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
  static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
  static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

  int coneNumber;
  
  @Override
  public void runOpMode() {
      int coneNumber = 0;
      // Initialize the drive system variables.
      frontleft = hardwareMap.get(DcMotor.class, "frontleft");
      frontright = hardwareMap.get(DcMotor.class, "frontright");
      rearleft = hardwareMap.get(DcMotor.class, "rearleft");
      rearright = hardwareMap.get(DcMotor.class, "rearright");
      left = hardwareMap.get(Servo.class, "left");
      right = hardwareMap.get(Servo.class, "right");
      lift4front = hardwareMap.get(DcMotor.class, "lift4front");
      lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
      vuforiaPOWERPLAY = new VuforiaCurrentGame();
      tfod = new Tfod();

      // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
      // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
      // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
      frontleft.setDirection(DcMotor.Direction.REVERSE);
      frontright.setDirection(DcMotor.Direction.FORWARD);
      rearleft.setDirection(DcMotor.Direction.REVERSE);
      rearright.setDirection(DcMotor.Direction.FORWARD);
      
      lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      // define initialization values for IMU, and then initialize it.
      imu = hardwareMap.get(IMU.class, "imu");
      RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
      RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
      
      RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
      
      imu.initialize(new IMU.Parameters(orientationOnRobot));
        
      // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rearleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rearright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      // Wait for the game to start (Display Gyro value while waiting)
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
      waitForStart();

      // Set the encoders for closed loop speed control
      frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

      resetHeading(); // reset heading

      
  }
  

  public void driveStraight(double maxDriveSpeed,
                            double distance,
                            double heading) {

      // Ensure that the opmode is still active
      if (opModeIsActive()) {

          // Determine new target position, and pass to motor controller
          int moveCounts = (int)(distance * COUNTS_PER_INCH);
          frontleftTarget = frontleft.getCurrentPosition() + moveCounts;
          frontrightTarget = frontright.getCurrentPosition() + moveCounts;
          rearleftTarget = rearleft.getCurrentPosition() + moveCounts;
          rearrightTarget = rearright.getCurrentPosition() + moveCounts;

          // Set Target FIRST, then turn on RUN_TO_POSITION
          frontleft.setTargetPosition(frontleftTarget);
          frontright.setTargetPosition(frontrightTarget);
          rearleft.setTargetPosition(rearleftTarget);
          rearright.setTargetPosition(rearrightTarget);

          frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

          // Set the required driving speed  (must be positive for RUN_TO_POSITION)
          // Start driving straight, and then enter the control loop
          maxDriveSpeed = Math.abs(maxDriveSpeed);
          moveRobot(maxDriveSpeed, 0);

          // keep looping while we are still active, and BOTH motors are running.
          //int tick = 0;
          while (opModeIsActive() &&
                  (frontleft.isBusy() && frontright.isBusy() && rearleft.isBusy() && rearright.isBusy())) {
              //tick += 1;
              //telemetry.addData("tick", tick);
              //telemetry.update();
              //Determine required steering to keep on heading
              turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

              // if driving in reverse, the motor correction also needs to be reversed
              if (distance < 0)
                  turnSpeed *= -1.0;

              // Apply the turning correction to the current driving speed.
              moveRobot(driveSpeed, 0);

              // Display drive status for the driver.
              sendTelemetry(true);

          }

          // Stop all motion & Turn off RUN_TO_POSITION
          moveRobot(0, 0);
          frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      }
      
  }

  public void turnToHeading(double maxTurnSpeed, double heading) {
      heading = -heading;
      // Run getSteeringCorrection() once to pre-calculate the current error
      getSteeringCorrection(heading, P_DRIVE_GAIN);

      // keep looping while we are still active, and not on heading.
      while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

          // Determine required steering to keep on heading
          turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

          // Clip the speed to the maximum permitted value.
          turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

          // Pivot in place by applying the turning correction
          moveRobot(0, turnSpeed);
          
          // Display drive status for the driver.
          sendTelemetry(false);

      }

      // Stop all motion;
      moveRobot(0, 0);

  }

  public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

      ElapsedTime holdTimer = new ElapsedTime();
      holdTimer.reset();

      // keep looping while we have time remaining.
      while (opModeIsActive() && (holdTimer.time() < holdTime)) {
          // Determine required steering to keep on heading
          turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

          // Clip the speed to the maximum permitted value.
          turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

          // Pivot in place by applying the turning correction
          moveRobot(0, turnSpeed);

          // Display drive status for the driver.
          sendTelemetry(false);
      }

      // Stop all motion;
      moveRobot(0, 0);
  }


  public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
      targetHeading = desiredHeading;  // Save for telemetry

      // Get the robot heading by applying an offset to the IMU heading
      robotHeading = getRawHeading() - headingOffset;

      // Determine the heading current error
      headingError =  targetHeading - robotHeading;

      // Normalize the error to be within +/- 180 degrees
      while (headingError > 180)  headingError -= 360;
      while (headingError <= -180) headingError += 360;

      // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
      return Range.clip(headingError * proportionalGain, -1, 1);
  }
  private void gripper(int status) {
    if (status == 0) {
      left.setPosition(0.66);
      right.setPosition(0.37);
    } else if (status == 1) {
      left.setPosition(0.02);
      right.setPosition(1);
    } else {
    }
    sleep(1000);
  }
  
  private void lift(int status, double speed) {
    int liftEncoderDistance = 0;
    if (status == 0) {
      liftEncoderDistance = 0;
    } else if (status == 1) {
      liftEncoderDistance = 610;
    } else if (status == 2) {
      liftEncoderDistance = 1075;
    } else if (status == 3) {
      liftEncoderDistance = 1460;
    } else if (status == 4) {
      lift4front.setTargetPosition(0);
      lift5rear.setTargetPosition(0);
    }
    while (Math.abs(lift5rear.getCurrentPosition()-liftEncoderDistance)>10 && opModeIsActive()) {
      lift4front.setPower(speed);
      lift5rear.setPower(speed);
      lift4front.setTargetPosition(-liftEncoderDistance);
      lift5rear.setTargetPosition(liftEncoderDistance);
      lift4front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      lift5rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      
      telemetry.addData("Lifting to", liftEncoderDistance);
      telemetry.addData("At Speed", speed);
      telemetry.addData("Currently at", lift5rear.getCurrentPosition());
      telemetry.update();
    }
  }
  
  private void strafe(double distance, double speed) {
    double strafeEncoderDistance;
    double strafeSpeed;

    strafeEncoderDistance = distance * 45.125;
    strafeSpeed = speed;
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setPower(-driveSpeed);
    frontright.setPower(driveSpeed);
    rearleft.setPower(driveSpeed);
    rearright.setPower(-driveSpeed);
    while (Math.abs(frontleft.getCurrentPosition()) < Math.abs(strafeEncoderDistance - 100) && opModeIsActive()) {
      telemetry.addData("Strafing to", strafeEncoderDistance);
      telemetry.addData("At Speed", strafeSpeed);
      telemetry.addData("Currently at:", frontleft.getCurrentPosition());
      telemetry.update();
    }
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
    sleep(500);
  }

  public void moveRobot(double drive, double turn) {
      driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
      turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

      frontleftSpeed  = drive - turn;
      frontrightSpeed = drive + turn;
      rearleftSpeed = drive - turn;
      rearrightSpeed = drive + turn;

      // Scale speeds down if either one exceeds +/- 1.0;
      double max = Math.max(Math.abs(frontleftSpeed), Math.abs(frontrightSpeed));
      if (max > 1.0)
      {
          frontleftSpeed /= max;
          frontrightSpeed /= max;
          rearleftSpeed /= max;
          rearrightSpeed /= max;
      }

      frontleft.setPower(frontleftSpeed);
      frontright.setPower(frontrightSpeed);
      rearleft.setPower(rearleftSpeed);
      rearright.setPower(rearrightSpeed);
  }


  private void sendTelemetry(boolean straight) {

      if (straight) {
          telemetry.addData("Motion", "Drive Straight");
      } else {
          telemetry.addData("Motion", "Turning");
      }

      telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
      telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
      telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", frontleftSpeed, frontrightSpeed);
      telemetry.update();
  }

  /**
   * read the raw (un-offset Gyro heading) directly from the IMU
   */
  public double getRawHeading() {
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      return orientation.getYaw(AngleUnit.DEGREES);
  }

  /**
   * Reset the "offset" heading back to zero
   */
  public void resetHeading() {
      // Save a new heading offset equal to the current raw heading.
      headingOffset = getRawHeading();
      robotHeading = 0;
  }
}
