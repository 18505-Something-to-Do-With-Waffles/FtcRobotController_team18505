package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.Backup_Robot;



@Autonomous(name = "Autonomous_Left", preselectTeleOp = "Main2022")
public class Autonomous_Left extends LinearOpMode {
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
        int cone = robot.readCone();
        telemetry.addLine("Reading Cone...");
        telemetry.update();
        telemetry.addData("Cone", cone);
        telemetry.update();


//        robot.driveToNewPos(36, 0.2);


//        robot.gripper(false);
//        robot.gripper(true);
//        robot.autoDriveDist(3,0.3);
//        robot.autoTurnToHeading(-38, 0.3, 0.5);
//        robot.autoSetLiftPosition(1);
//        robot.autoDriveDist(3, 0.3);
//        sleep(200);
//        robot.gripper(false);
//        sleep(400);
//        robot.autoSetLiftPosition(0);
//        robot.autoDriveDist(-3, 0.3);
//        robot.autoTurnToHeading(0, 0.3, 0.5);
//        robot.autoDriveDist(-3, 0.3);
//
//        robot.autoDriveDist(54, 0.2);
//        robot.autoDriveDist(-4, 0.2);
//
//        if (cone == 1) {
//            robot.autoTurnToHeading(-90, 0.2, 0.5);
//            robot.autoDriveDist(24, 0.2);
//        }
//        if (cone == 3) {
//            robot.autoTurnToHeading(90, 0.2, 0.5);
//            robot.autoDriveDist(24, 0.2);
//        }





    }
}