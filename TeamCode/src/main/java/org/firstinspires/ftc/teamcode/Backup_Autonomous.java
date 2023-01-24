package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.Backup_Robot;


@Disabled
@Autonomous(name = "Backup_Autonomous")
public class Backup_Autonomous extends LinearOpMode {

    public void runOpMode() {

        // Instantiate stuff
        Backup_Robot robot;


        // Initialize stuff
        robot = new Backup_Robot(hardwareMap, telemetry);


        // Wait for start
        waitForStart();


        // Do stuff
        robot.resetHeading();
        while (opModeIsActive()) {
            telemetry.addData("Heading: ", robot.getHeading());
            telemetry.update();
            sleep(1000);
        }
    }
}