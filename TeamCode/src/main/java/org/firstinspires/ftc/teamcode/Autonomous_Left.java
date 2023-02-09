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

        robot.gripper(false);
        robot.gripper(true);
        robot.autoDriveDist(3,0.3);
        robot.autoTurnToHeading(45, 0.3, 0.5);
        robot.setLiftPos(1);
        robot.autoDriveDist(7.2, 0.3);
        robot.gripper(false);
        robot.setLiftPos(0);
        robot.autoDriveDist(-7.2, 0.3);
        robot.autoTurnToHeading(-45, 0.3, 0.5);
        robot.autoDriveDist(0.3, -3);
    }
}