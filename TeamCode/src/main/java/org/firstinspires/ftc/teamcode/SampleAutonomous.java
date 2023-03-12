package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.drive.WaffleDrive;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Autonomous(name = "SAMPLE-AUTO", preselectTeleOp = "Main2022")
public class SampleAutonomous extends LinearOpMode {
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff
        Gripper gripper;
        Lift lift;
        Vision vision;

        // Initialize stuff
        WaffleDrive drive = new WaffleDrive(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        vision = new Vision(hardwareMap, new String[]{"blue", "green", "red"}, 1.6, 1, "model_20221228_153128.tflite");

        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        // Do stuff
        int cone = vision.readCone();
        telemetry.addLine("Reading Cone...");
        telemetry.update();
        telemetry.addData("Cone", cone);
        telemetry.update();

    }
}