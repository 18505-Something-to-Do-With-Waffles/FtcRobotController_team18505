package org.firstinspires.ftc.teamcode;


class Gyro {
    public void initialize() {

    }
    public double getRawHeading() {
        return 0.0;
    }
    public void resetHeading() {

    }
}

class DistSensor {
    public void initialize() {

    }
    public double getDist() {
        return 0.0;
    }
}

class Robot {
    public void initialize() {

    }
    public void turn(double turnSpeed, double degrees) {

    }
    public void drive(double driveSpeed, double distance) {

    }
    public void strafe(double strafeSpeed, double distance) {

    }
    public void grip(boolean open) {

    }
    public void lift(int level, double liftSpeed) {

    }

}

class DeadwheelEncoder {
    public void initialize() {

    }
    public void reset() {

    }
    public double getDist() {
        return 0.0;
    }
}


public class AutoSkeleton {

    // Instantiate Objects here
    Gyro gyro = new Gyro();
    Robot robot = new Robot();
    DeadwheelEncoder fwdEncoder = new DeadwheelEncoder();
    DeadwheelEncoder strafeEncoder = new DeadwheelEncoder();
    // other objects to instantiate?




    public void initialize() {
        // Initialize objects here
        gyro.initialize();
        robot.initialize();
        fwdEncoder.initialize();
        strafeEncoder.initialize();
        // other objects to initialize?

    }
    public void main() {
        initialize();

    }
}
