package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class TeleOpControl {

    Gripper gripper;
    Lift lift;

    public TeleOpControl(HardwareMap hardwareMap) {
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
    }
    public void gripperTeleOp(Controller controller) {
        Boolean open = false;
        if (controller.BOnce()){
            if (open){
                open = false;
                gripper.open();
            }
            else{
                open = true;
                gripper.close();
            }
        }
    }

    public void liftTeleOp(Controller controller) {
        if (controller.dpadUpOnce()) {
            if(controller.dpadUpOnce()){
                lift.up();
            }
            else if (controller.dpadDownOnce()) {
                lift.down();
            }
        }
    }

}
