package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main2022")
public class Main2022 extends LinearOpMode {

    private DcMotor lift5rear; //rear lift motor
    private DcMotor lift4front; //front lift motor
    private DcMotor rearleft; //rear left drive motor
    private DcMotor frontleft; //front left drive motor
    private DcMotor frontright; //front right drive motor
    private DcMotor rearright; //rear right drive motor
    private Servo left; //left gripper servo
    private Servo right; //right gripper servo

    int liftPos; //position of lift 0-4
    int DpadDownLastTick;
    int DpadUpLastTick;

    // lift functionality
    private void lift() {

        telemetry.addData("lift position", liftPos);
        telemetry.addData("dp down lt", DpadDownLastTick);
        telemetry.addData("dp up lt", DpadUpLastTick);
        if (gamepad1.dpad_right && lift5rear.getTargetPosition() < 1480) {
            liftPos = 4;
            lift4front.setTargetPosition(lift4front.getTargetPosition() - 20);
            lift5rear.setTargetPosition(lift5rear.getTargetPosition() + 20);
        } else if (gamepad1.dpad_left && lift5rear.getTargetPosition() > 5) {
            liftPos = 4;
            lift4front.setTargetPosition(lift4front.getTargetPosition() + 20);
            lift5rear.setTargetPosition(lift5rear.getTargetPosition() - 20);
        }
        if (!gamepad1.dpad_down) {
            DpadDownLastTick = 0;
        }
        if (!gamepad1.dpad_up) {
            DpadUpLastTick = 0;
        }
        if (gamepad1.dpad_down && (liftPos == 3 || liftPos == 4)) {
            liftPos = 0;
            DpadDownLastTick = 1;
        } else if (gamepad1.dpad_down && (liftPos == 2 || liftPos == 4) && DpadDownLastTick == 0) {
            liftPos = 0;
            DpadDownLastTick = 1;
        } else if (gamepad1.dpad_down && (liftPos == 1 || liftPos == 4) && DpadDownLastTick == 0) {
            liftPos = 0;
            DpadDownLastTick = 1;
        } else if (gamepad1.dpad_up && (liftPos == 0 || liftPos == 4)) {
            liftPos = 1;
            DpadUpLastTick = 1;
        } else if (gamepad1.dpad_up && (liftPos == 1 || liftPos == 4) && DpadUpLastTick == 0) {
            liftPos = 2;
            DpadUpLastTick = 1;
        } else if (gamepad1.dpad_up && (liftPos == 2 || liftPos == 4) && DpadUpLastTick == 0) {
            liftPos = 3;
            DpadUpLastTick = 1;
        }
        if (liftPos == 0) {
            lift4front.setTargetPosition(0);
            lift5rear.setTargetPosition(0);
        } else if (liftPos == 1) {
            lift4front.setTargetPosition(-610);
            lift5rear.setTargetPosition(610);
        } else if (liftPos == 2) {
            lift4front.setTargetPosition(-1075);
            lift5rear.setTargetPosition(1075);
        } else if (liftPos == 3) {
            lift4front.setTargetPosition(-1460);
            lift5rear.setTargetPosition(1460);
        }
        telemetry.addData("lift motor position", lift5rear.getCurrentPosition());
        telemetry.addData("lift motor position2", lift4front.getCurrentPosition());
        if (lift5rear.getCurrentPosition() <= 10 && lift5rear.getCurrentPosition() >= -10 && liftPos == 0) {
            lift4front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift5rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift4front.setPower(0);
            lift5rear.setPower(0);
        } else {
            lift4front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift5rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift4front.setPower(-0.5);
            lift5rear.setPower(0.5);
        }
    }

    //Driving functionality
    private void drive() {
        double speedMultiplier; //max speed from 0-1
        double movementy; //movement along y axis
        double movementx; //movement along x axis
        double movementr; //rotational movement
        //int lengthPressed; //consecutive ticks that a player has driven

        if (!(liftPos == 0)) { //if lift is up
            speedMultiplier = 0.3;
        } else { //if lift is down
            speedMultiplier = 0.85;
        }
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            //lengthPressed += 1;
        }
        else {
            //lengthPressed = 0;
        }
        movementy = gamepad1.left_stick_y * speedMultiplier;
        movementx = gamepad1.left_stick_x * speedMultiplier;
        movementr = gamepad1.right_stick_x * speedMultiplier * 1.5;
        rearleft.setPower((movementy + movementx) - movementr);
        frontleft.setPower((movementy + (0 - movementx)) - movementr);
        frontright.setPower(((0 - movementy) + (0 - movementx)) - movementr);
        rearright.setPower(((0 - movementy) + movementx) - movementr);
    }

    //gripper functionality
    private void gripper() {
        if (gamepad1.a) {
            left.setPosition(0.02);
            right.setPosition(1);
        }
        if (gamepad1.b) {
            left.setPosition(0.66);
            right.setPosition(0.37);
        }
        telemetry.addData("left position", left.getPosition());
    }

    @Override
    public void runOpMode() {
        lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
        lift4front = hardwareMap.get(DcMotor.class, "lift4front");
        rearleft = hardwareMap.get(DcMotor.class, "rearleft");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        rearright = hardwareMap.get(DcMotor.class, "rearright");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        lift4front.setTargetPosition(0);
        lift5rear.setTargetPosition(0);
        waitForStart();
        liftPos = 0;
        lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift4front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift5rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.update();
                drive();
                gripper();
                lift();
            }
        }
    }


}