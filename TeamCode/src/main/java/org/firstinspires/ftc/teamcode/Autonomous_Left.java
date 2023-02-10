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
        robot.coneStackSetLiftPos(5);
        sleep(500);
        robot.gripper(true);
        sleep(500);
        robot.autoSetLiftPosition(2);

//        robot.gripper(false);
//        robot.gripper(true);
//        robot.autoDriveDist(3,0.3);
//        robot.autoTurnToHeading(-43, 0.3, 0.5);
//        robot.autoSetLiftPosition(1);
//        robot.autoDriveDist(3, 0.3);
//        sleep(500);
//        robot.gripper(false);
//        sleep(1000);
//        robot.autoSetLiftPosition(0);
//        robot.autoDriveDist(-3, 0.3);
//        robot.autoTurnToHeading(0, 0.3, 0.5);
//        robot.autoDriveDist(-3, 0.3);


    }
}