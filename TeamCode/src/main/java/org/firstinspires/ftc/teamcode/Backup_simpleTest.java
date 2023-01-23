package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.util.Log;

@TeleOp(name = "simpleTest")
public class Backup_simpleTest extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int i;

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            i = 0;
            while (opModeIsActive() && i < 5) {
                // Put loop blocks here.
                telemetry.addData("i = ", i);
                telemetry.update();
                Log.d("Sleep", "About to sleep"); // can write to Logcat, but may get lost in the noise
                sleep(1000);
                i += 1;
                telemetry.update();
            }
        }
    }
}