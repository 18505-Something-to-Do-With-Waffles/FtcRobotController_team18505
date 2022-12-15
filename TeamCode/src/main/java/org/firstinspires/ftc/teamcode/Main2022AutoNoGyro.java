package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Main2022AutoNoGyro")
public class Main2022AutoNoGyro extends LinearOpMode {
    private DcMotor lift5rear;
    private DcMotor lift4front;
    private DcMotor rearleft;
    private DcMotor frontleft;
    private DcMotor frontright;
    private DcMotor rearright;
    private Servo left;
    private Servo right;
    double countsPerInch = 446.94;


    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        rearright = hardwareMap.get(DcMotor.class, "rearright");

        frontleft.setDirection(DcMotor.Direction.REVERSE);
        rearleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        rearright.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            drive(24, 0.4);
        }


    }
    public void drive(double distance, double maxSpeed) {

        int countDistance = (int) (distance * countsPerInch);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int target = frontleft.getCurrentPosition() + countDistance;

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleft.setPower(maxSpeed);
        frontright.setPower(maxSpeed);
        rearleft.setPower(maxSpeed);
        rearright.setPower(maxSpeed);

        while (opModeIsActive() && frontleft.isBusy()) {
            if (frontleft.getCurrentPosition() > target - 50 && frontleft.getCurrentPosition() < target + 50) {
                frontleft.setPower(0);
                frontright.setPower(0);
                rearleft.setPower(0);
                rearright.setPower(0);
            }
        }
    }


    public void strafe(double distance, double speed) {
            int strafeCountDistance = (int) (distance * countsPerInch);

            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontleft.setTargetPosition(frontleft.getCurrentPosition() + strafeCountDistance);
            frontright.setTargetPosition(-(frontright.getCurrentPosition() + strafeCountDistance));
            rearleft.setTargetPosition(-(rearleft.getCurrentPosition() + strafeCountDistance));
            rearright.setTargetPosition(rearright.getCurrentPosition() + strafeCountDistance);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleft.setPower(speed);
            frontright.setPower(speed);
            rearleft.setPower(speed);
            rearright.setPower(speed);

            while (opModeIsActive() && frontleft.isBusy() && frontright.isBusy() && rearleft.isBusy() && rearright.isBusy()) {
                if (frontleft.getCurrentPosition() > frontleft.getTargetPosition() - 50 || frontleft.getCurrentPosition() < frontleft.getTargetPosition() + 50){
                }
            }

    }
}
