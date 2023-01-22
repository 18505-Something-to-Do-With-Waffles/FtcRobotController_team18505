package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "RightSideAuto2022", preselectTeleOp = "Main2022")
public class RightSideAuto2022 extends LinearOpMode {

  private DcMotor frontleft;
  private DcMotor frontright;
  private DcMotor rearleft;
  private DcMotor rearright;
  private Servo left;
  private Servo right;
  private DcMotor lift4front;
  private DcMotor lift5rear;
  private VuforiaCurrentGame vuforiaPOWERPLAY;
  private Tfod tfod;

  double driveSpeed;

  /**
   * Describe this function...
   */
  private void pivot(int degrees, double speed) {
    double pivotEncoderDistance;
    double pivotSpeed;

    pivotEncoderDistance = 9.5 * degrees;
    pivotSpeed = speed;
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setPower(pivotSpeed);
    frontright.setPower(-pivotSpeed);
    rearleft.setPower(pivotSpeed);
    rearright.setPower(-pivotSpeed);
    while (Math.abs(frontleft.getCurrentPosition()) < Math.abs(pivotEncoderDistance - 100) && opModeIsActive()) {
      telemetry.addData("Pivoting to:", pivotEncoderDistance);
      telemetry.addData("At Speed", pivotSpeed);
      telemetry.addData("Currently at:", frontleft.getCurrentPosition());
      telemetry.update();
    }
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
    sleep(500);
  }

  /**
   * Describe this function...
   */
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

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
   
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

  /**
   * Describe this function...
   */
  private void drive(double distance, double speed) {
    double driveEncoderDistance;

    driveEncoderDistance = 45.2849 * distance;
    driveSpeed = speed;
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setPower(driveSpeed);
    frontright.setPower(driveSpeed);
    rearleft.setPower(driveSpeed);
    rearright.setPower(driveSpeed);
    while (Math.abs(frontleft.getCurrentPosition()) < Math.abs(driveEncoderDistance - 100) && opModeIsActive()) {
      telemetry.addData("Driving to:", driveEncoderDistance);
      telemetry.addData("At Speed", driveSpeed);
      telemetry.addData("Currently at:", frontleft.getCurrentPosition());
      telemetry.update();
    }
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
    sleep(500);
  }

  /**
   * Describe this function...
   */
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

  /**
   * Describe this function...
   */
  private void cameraInit() {
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
    // Set isModelTensorFlow2 to true if you used a TensorFlow
    // 2 tool, such as ftc-ml, to create the model.
    //
    // Set isModelQuantized to true if the model is
    // quantized. Models created with ftc-ml are quantized.
    //
    // Set inputSize to the image size corresponding to the model.
    // If your model is based on SSD MobileNet v2
    // 320x320, the image size is 300 (srsly!).
    // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
    // If your model is based on SSD MobileNet V1 FPN 640x640 or
    // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
    tfod.useModelFromFile("model_20221228_153128.tflite", JavaUtil.createListWith("blue", "green", "red"), true, true, 320);
    tfod.initialize(vuforiaPOWERPLAY, (float) 0.55, true, true);
    tfod.setClippingMargins(0, 0, 0, 0);
    tfod.activate();
    // Enable following block to zoom in on target.
    tfod.setZoom(1.6, 1 / 1);
  }
  private int readCone() {
    int colorIterable;
    int cameraReads = 0;
    List<Recognition> recognitions;
    Recognition recognition;
    int lastread = 0;
    int colorConfirm = 0;
    double highestConfidence = 0;
    if(vuforiaPOWERPLAY == null){
      telemetry.addData("vf","true");
      telemetry.update();
    }

    
    telemetry.addData("init", 2);
    telemetry.update();
    resetRuntime();
    cameraReads = 2;
    while (getRuntime()<2 && opModeIsActive()) {
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

  @Override
  public void runOpMode() {
    double coneNumber;

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

    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
    lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    telemetry.addData("init", 1);
    
    cameraInit();
    telemetry.update();
    waitForStart();
    coneNumber = readCone();
    telemetry.addData("Reading Cone", coneNumber);
    telemetry.update();
    
    gripper(0);
    gripper(1);
    drive(3,0.3);
    pivot(45,-0.3);
    lift(1,0.5);
    drive(7.2,0.3);
    gripper(0);
    lift(0,0.5);
    drive(7.2,-0.3);
    pivot(45,0.3);
    drive(3,-0.3);
    
    if (coneNumber == 1) {
      drive(27, 0.3);
      pivot(90, 0.3);
      drive(23, -0.3);
      gripper(0);
    } else if (coneNumber == 2) {
      drive(33, 0.3);
    } else if (coneNumber == 3) {
      drive(27, 0.3);
      pivot(90, 0.3);
      drive(15, 0.3);
      gripper(0);
      drive(6.5, 0.3);
    }
    

    vuforiaPOWERPLAY.close();
    tfod.close();
  }
  


  
}
