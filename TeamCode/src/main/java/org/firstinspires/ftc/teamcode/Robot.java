package org.firstinspires.ftc.teamcode;

// For hardwareMap and telemetry
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
// For Controller
import com.qualcomm.robotcore.hardware.Gamepad;
// For IMU
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

// For Distance Sensor
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;
import java.lang.*;

public class Robot {

    // Instantiate Telemetry
    private Telemetry telemetry;
    private Intake intake;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }
}

class Intake {
    static public double speed;
    static void Intake(double motorSpeed) {
        speed = motorSpeed;
    }
    private void on() {
        // TODO: DC Motor Set to speed
    }
    private void off() {
        // TODO: DC Motor set to 0
    }
}

class Lift {
    static public double speed;
    static void Lift (double motorSpeed) {
        speed = motorSpeed;
    }
    void up() {
        // TODO: Set lift motors to speed
    }
    void down() {
        // TODO: Set Lift motors to -speed
    }
    void setPosition() {
        // TODO: up() or down() until position is correct
    }
}

class Launcher {
    static public double speed;
    static void Launcher (double motorSpeed) {
        speed = motorSpeed;
    }
}
