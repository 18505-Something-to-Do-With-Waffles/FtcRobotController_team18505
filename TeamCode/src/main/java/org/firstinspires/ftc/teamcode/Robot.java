package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// For Camera
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

public class Robot {

    // Instantiate Telemetry
    private Telemetry telemetry;

    // Instantiate Robot Systems
    private Intake intake;
    private Lift lift;
    private Launcher launcher;
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap, 0.5);
        launcher = new Launcher(hardwareMap);
    }
}

class Intake {
    private DcMotor intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
    }
    public void on(double speed) {
        intake.setPower(speed);
    }
    public void off() {
        intake.setPower(0);
    }
}

class Lift {
    private DcMotor lift;
    double power;
    public Lift (HardwareMap hardwareMap, double globalPower) {
        // Initializing lift motor and power
        lift = hardwareMap.get(DcMotor.class, "lift");
        power = globalPower;

        // Resetting Encoder of lift motor and setting it to "RUN_TO_POSITION" mode
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void up(int speed) {
        lift.setTargetPosition(lift.getTargetPosition()+speed);
    }
    void down(int speed) {
        lift.setTargetPosition(lift.getTargetPosition()-speed);
    }
    void off() {
        lift.setPower(0);
    }
    void setPosition(int encoderPosition) {
        lift.setTargetPosition(encoderPosition);
    }
    void checkForZero() {
        if (lift.getTargetPosition() == 0 && Math.abs(lift.getCurrentPosition()) < 10) {
            lift.setPower(0);
        }
        else {
            lift.setPower(power);
        }
    }
}

class Launcher {
    DcMotor launch;
    public Launcher (HardwareMap hardwareMap) {
        launch = hardwareMap.get(DcMotor.class, "launch");
    }
    public void shoot (double speed) {
        launch.setPower(speed);
    }
    public void off (double speed) {
        launch.setPower(0);
    }
}
