package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.Backup_Robot;


@Disabled
@Autonomous(name = "Autonomous_Left")
public class Autonomous_Left extends LinearOpMode {
    private char side = 'L';
    public void runOpMode() {

        // Instantiate stuff
        Backup_Robot robot;


        // Initialize stuff
        robot = new Backup_Robot(hardwareMap, telemetry);


        // Wait for start
        waitForStart();


        // Do stuff
        robot.resetHeading();

    }
}