package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    // instantiate lift motor
    private DcMotor rearLift;
    private DcMotor frontLift;

    // define lift positions
    final private int[] polePosList = {0, 610, 1075, 1460};
    final private  int[] coneStackPos = {0, 45, 90, 135, 180, 360};

    public int polePosition;

    public Lift(HardwareMap hardwareMap){
        frontLift = hardwareMap.get(DcMotor.class, "lift4front");
        rearLift = hardwareMap.get(DcMotor.class, "lift5rear");
    }

    public void setPosition(int newPos, double power){
        frontLift.setPower(power);
        rearLift.setPower(power);
        rearLift.setTargetPosition(newPos);
        frontLift.setTargetPosition(-newPos);
    }

    public void setPolePosition(int newPolePos){
        this.setPosition(polePosList[newPolePos], 1);
        polePosition = newPolePos;
    }

    public void checkForZero(){
        if (polePosition == 0 && Math.abs(rearLift.getCurrentPosition()) < 10){
            frontLift.setPower(0);
            rearLift.setPower(0);
        }
    }

    public void up(){
        if (polePosition < polePosList.length - 1){
            this.setPolePosition(polePosition + 1);
        }
    }

    public void down(){
        if (polePosition > 0){
            this.setPolePosition(0);
        }
    }
    
}
