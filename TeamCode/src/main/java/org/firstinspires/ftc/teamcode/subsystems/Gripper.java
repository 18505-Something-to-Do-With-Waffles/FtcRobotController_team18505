package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {

    // instantiate gripper servos
    private Servo left;
    private Servo right;

    // set close() and open() positions
    private double[] closePosition = {0.02, 1};
    private double[] openPosition = {0.66, 0.37};

    public Gripper (HardwareMap hardwareMap){
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
    }

    public void close(){
        left.setPosition(closePosition[0]);
        right.setPosition(closePosition[1]);
    }

    public void open(){
        left.setPosition(openPosition[0]);
        right.setPosition(openPosition[1]);
    }
}
