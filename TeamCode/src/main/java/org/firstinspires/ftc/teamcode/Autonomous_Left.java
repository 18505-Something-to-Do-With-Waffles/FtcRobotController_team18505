package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.Backup_Robot;



@Autonomous(name = "Autonomous_Left")
public class Autonomous_Left extends LinearOpMode {
    private char side = 'L';
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff
        Robot robot;


        // Initialize stuff
        robot = new Robot(hardwareMap, telemetry);


        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();


        // Do stuff
        robot.resetHeading();
        telemetry.addLine("Reading Cone...");
        telemetry.update();
        telemetry.addData("Cone", robot.readCone());
        telemetry.update();
        sleep(3000);
        robot.autoDriveDist(24,0.3);
        sleep(500);
        robot.autoDriveDist(-24,0.3);
        sleep(500);
        robot.autoTurnToHeading(90,0.15);
        sleep(500);
        robot.autoTurnToHeading(-90,0.15);
        sleep(500);
        robot.setLiftPos(3);
        sleep(5000);
        robot.setLiftPos(0);
        sleep(500);
        robot.gripper(true);
        sleep(500);
        robot.gripper(false);
    }
}