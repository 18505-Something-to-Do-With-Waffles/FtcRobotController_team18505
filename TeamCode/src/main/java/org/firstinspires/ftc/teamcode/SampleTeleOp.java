package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Sample TeleOp")
public class SampleTeleOp extends LinearOpMode {
    TeleOpControl control;
    Controller controller1;

    public SampleTeleOp(){
        control = new TeleOpControl(hardwareMap);
        controller1 = new Controller(gamepad1);
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("Ready For TeleOp!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            control.gripperTeleOp(controller1);
            control.liftTeleOp(controller1);
        }
    }
}
