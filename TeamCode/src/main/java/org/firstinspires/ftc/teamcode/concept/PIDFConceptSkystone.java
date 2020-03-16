package org.firstinspires.ftc.teamcode.concept;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "Concept PIDF", group = "Concept")
public class PIDFConceptSkystone extends LinearOpMode {
    // Motor
    DcMotorEx testMotor;
    // PID Coefficients
    public static final double k_P = 2.5;
    public static final double k_I = 0.1;
    public static final double k_D = 0.2;
    public static final double k_F = 0f;

    @Override
    public void runOpMode() {
        testMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_drive");

        waitForStart();

        PIDFCoefficients pidOrig = testMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidNew = new PIDFCoefficients(k_P, k_I, k_D, k_F);
        testMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients pidModified = testMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D,F (orig)", "%.04f, %.04f, %.04f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d, pidOrig.f);
            telemetry.addData("P,I,D,F (modified)", "%.04f, %.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d, pidModified.f);
            telemetry.update();
        }
    }
}
