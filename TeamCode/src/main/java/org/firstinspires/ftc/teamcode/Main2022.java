package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main2022")
public class Main2022 extends LinearOpMode {

    private DcMotor lift5rear;
    private DcMotor lift4front;
    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private Servo left;
    private Servo right;
    
    int DpadDownLastTick = 0;
    int DpadUpLastTick = 0;
    int liftPos;

    /**
     * Describe this function...
     */
    private void lift() {

        if (gamepad1.right_bumper){
            lift5rear.setPower(0);
            lift4front.setPower(0);
            lift4front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift5rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lift4front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift5rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift4front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift5rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        
        
        if (gamepad1.dpad_right && lift5rear.getTargetPosition() < 1480) {
            liftPos = 4;
            lift4front.setTargetPosition(lift4front.getTargetPosition() - 20);
            lift5rear.setTargetPosition(lift5rear.getTargetPosition() + 20);
        } else if (gamepad1.dpad_left) {
            liftPos = 4;
            lift4front.setTargetPosition(lift4front.getTargetPosition() + 20);
            lift5rear.setTargetPosition(lift5rear.getTargetPosition() - 20);
        }
        if (!gamepad1.dpad_down) {
            DpadDownLastTick = 0;
        }
        if(!gamepad1.dpad_up) {
            DpadUpLastTick = 0;
        }
        else{
            
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
        telemetry.addData("lift position", liftPos);
        telemetry.addData("dp down lt", DpadDownLastTick);
        telemetry.addData("dp up lt", DpadUpLastTick);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        lift5rear = hardwareMap.get(DcMotor.class, "lift5rear");
        lift4front = hardwareMap.get(DcMotor.class, "lift4front");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        // Put initialization blocks here.
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

                drive();
                gripper();
                lift();
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void drive() {
        double speedMultiplier;
        double movementy;
        double movementx;
        double movementr;

        if (!(liftPos == 0)) {
            speedMultiplier = 0.3;
        } else {
            speedMultiplier = 0.7;
        }
        movementy = gamepad1.left_stick_y * speedMultiplier;
        movementx = gamepad1.left_stick_x * speedMultiplier;
        movementr = gamepad1.right_stick_x * speedMultiplier * 1.5;
        leftRear.setPower((movementy + movementx) - movementr);
        leftFront.setPower((movementy + (0 - movementx)) - movementr);
        rightFront.setPower(((0 - movementy) + (0 - movementx)) - movementr);
        rightRear.setPower(((0 - movementy) + movementx) - movementr);
    }

    /**
     * Describe this function...
     */
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
}
