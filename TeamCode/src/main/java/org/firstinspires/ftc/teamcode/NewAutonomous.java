package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import org.firstinspires.ftc.teamcode.Backup_Robot;



@Autonomous(name = "NewAutonomous", preselectTeleOp = "Main2022")
public class NewAutonomous extends LinearOpMode {
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff
        Robot robot;


        // Initialize stuff
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 10), 0)
                .splineTo(new Vector2d(10, 3), 0)
                .build();



        // Wait for start
        telemetry.addLine("Initialized!");
        telemetry.update();
        waitForStart();

        if(isStopRequested()) return;

        // Do stuff
        robot.resetHeading();
        int cone = robot.readCone();
        telemetry.addLine("Reading Cone...");
        telemetry.update();
        telemetry.addData("Cone", cone);
        telemetry.update();

        drive.followTrajectory(myTrajectory);
        robot.gripper(true);
        robot.autoSetLiftPosition(3);
        robot.gripper(false);
        robot.autoSetLiftPosition(0);
        drive.followTrajectory(myTrajectory);

    }
}