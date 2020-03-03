package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Mecanum Test Chassis", group = "Test")
public class Test_Chassis extends OpMode {
    // Creating the robot
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        // IMU Angular Readings
        telemetry.addData("Roll", robot.angles.secondAngle);
        telemetry.addData("Pitch", robot.angles.thirdAngle);
        telemetry.addData("Heading/Yaw", robot.angles.firstAngle);

        robot.mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }
}
