package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.src.Robot;

@Disabled
@Autonomous(name = "Base Autonomous", group = "Autonomous")
public class Base_Auto extends LinearOpMode {
    // Create the robot
    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        // Initialize Robot
        robot.init(hardwareMap);
        // Wait For Start To Be Pressed
        waitForStart();
        // Place all other commands below:
    }
}
