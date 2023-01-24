package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled
@TeleOp(name = "Backup_TeleOp")
public class Backup_TeleOp extends LinearOpMode {

    public void runOpMode() {

        // Instantiate stuff
        Backup_Robot robot;
        Controller controller1;


        // Initialize stuff
        robot = new Backup_Robot(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);


        // Wait for start
        waitForStart();


        // Do stuff
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                telemetry.update();
                robot.teleDrive(controller1);
                //robot.teleGripper();
                //robot.teleLift();

            }
        }
    }
}