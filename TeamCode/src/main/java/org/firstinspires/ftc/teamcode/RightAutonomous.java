package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import org.firstinspires.ftc.teamcode.Backup_Robot;

@Autonomous(name = "RIGHT-AUTO", preselectTeleOp = "Main2022")
public class RightAutonomous extends LinearOpMode {
    public void runOpMode() {

        telemetry.addLine("Initializing...");
        telemetry.update();
        // Instantiate stuff
        Robot robot;

        // Initialize stuff
        robot = new Robot(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Pose2d startPose = new Pose2d(-37.8, 61.6, 4.712);
//        drive.setPoseEstimate(startPose);

        Trajectory lineUpFirstPole = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(5.289, 6.04, 0.872665))
                .build();
        Trajectory firstPole = drive.trajectoryBuilder(lineUpFirstPole.end())
                .lineToLinearHeading(new Pose2d(8.0, 9.0, 0.88))
                .build();
        Trajectory lineUpBeforeStraight = drive.trajectoryBuilder(firstPole.end())
                .lineToLinearHeading(new Pose2d(3, 5, 0))
                .build();
        Trajectory longStraight = drive.trajectoryBuilder(lineUpBeforeStraight.end())
                .lineToLinearHeading(new Pose2d(49.66, 3.1, 0))
                .build();
        Trajectory coneStack1 = drive.trajectoryBuilder(longStraight.end())
                .lineToLinearHeading(new Pose2d(49.44, -17.16, 4.720))
                .build();
        Trajectory backupToPole = drive.trajectoryBuilder(coneStack1.end())
                .lineToLinearHeading(new Pose2d(49.0, 1.88, 4.716))
                .build();
        Trajectory smallPole = drive.trajectoryBuilder(backupToPole.end())
                .lineToLinearHeading(new Pose2d(40.9, 3.05, 4.435))
                .build();
        Trajectory backupAfterPole = drive.trajectoryBuilder(smallPole.end())
                .lineToLinearHeading(new Pose2d(55, 4, 4.139))
                .build();
        Trajectory coneStack2 = drive.trajectoryBuilder(backupAfterPole.end())
                .lineToLinearHeading(new Pose2d(49.44, -16, 4.720))
                .build();
        Trajectory park1 = drive.trajectoryBuilder(coneStack2.end())
                .lineToLinearHeading(new Pose2d(49.6, 28.2, 4.716))
                .build();
        Trajectory park2 = drive.trajectoryBuilder(coneStack2.end())
                .lineToLinearHeading(new Pose2d(44.0, 1.8, 4.72))
                .build();
//        Trajectory park3 = drive.trajectoryBuilder(coneStack2.end())
//                .lineToLinearHeading(new Pose2d(49.44, -17.16, 4.720))
//                .build();
        Trajectory turnToPole = drive.trajectoryBuilder(coneStack2.end())
                .lineToLinearHeading(new Pose2d(46.82, -14, 2.47))
                .build();
        Trajectory backupAfterPull = drive.trajectoryBuilder(turnToPole.end())
                .back(1)
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

        robot.gripper(true);
        sleep(600);
        robot.setLiftPos(1);
        drive.followTrajectory(firstPole);
        robot.gripper(false);
        sleep(600);
        drive.followTrajectory(lineUpBeforeStraight);
        robot.setLiftPos(0);
        drive.followTrajectory(longStraight);
        robot.coneStackSetLiftPos(5);
        robot.gripper(false);
        drive.followTrajectory(coneStack1);
        robot.gripper(true);
        sleep(150);
        robot.setLiftPos(1);
        sleep(300);
        drive.followTrajectory(turnToPole);
        robot.gripper(false);
        sleep(300);
        drive.followTrajectory(coneStack2);
        robot.coneStackSetLiftPos(4);
        robot.gripper(true);
        sleep(150);
        robot.setLiftPos(1);
        sleep(300);
        drive.followTrajectory(turnToPole);
        robot.gripper(false);
        sleep(300);
        drive.followTrajectory(backupAfterPull);
        drive.followTrajectory(coneStack2);
        robot.coneStackSetLiftPos(3);
        robot.gripper(true);
        sleep(150);
        robot.setLiftPos(1);
        sleep(300);
        drive.followTrajectory(turnToPole);
        robot.gripper(false);
        sleep(300);
        drive.followTrajectory(coneStack2);
        robot.autoSetLiftPosition(0);

        switch (cone){
            case 1:
                drive.followTrajectory(park1);
                break;
            case 2:
                drive.followTrajectory(park2);
                break;
        }

    }
}