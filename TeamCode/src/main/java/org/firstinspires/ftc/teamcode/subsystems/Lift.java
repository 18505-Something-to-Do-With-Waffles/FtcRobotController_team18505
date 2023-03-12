package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    // instantiate lift motor
    private DcMotor lift5rear;
    private DcMotor lift4front;

    // define lift positions
    final private int[] polePos = {0, 610, 1075, 1460};
    final private  int[] coneStackPos = {0, 45, 90, 135, 180, 360};

    public Lift(HardwareMap hardwareMap){
        lift4front = hardwareMap.get(DcMotor.class, "lift4front");
        lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
    }

    public void setPosition(int newPos){

    }
}
