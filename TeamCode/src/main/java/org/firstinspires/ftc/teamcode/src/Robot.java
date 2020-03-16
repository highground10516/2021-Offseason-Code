package org.firstinspires.ftc.teamcode.src;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.teamcode.src.Constants.*;

public class Robot {
    // Robot Hardware
    // Drive Motors
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx[] driveMotors;
    // Inertial Motion Unit
    public BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /**
     * Initializes Robot Hardware Components
     * @param hMap Hardware Mapping Type
     */
    public void init(HardwareMap hMap) {
        // Drive
        frontLeft = hMap.get(DcMotorEx.class, "Front Left");
        frontRight = hMap.get(DcMotorEx.class, "Front Right");
        backLeft = hMap.get(DcMotorEx.class, "Back Left");
        backRight = hMap.get(DcMotorEx.class, "Back Right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        // REV Inertial Movement Unit (Gyro, Accelerometer, Magnetometer, etc.)
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        angles.firstAngle *= -1;
        angles.secondAngle *= -1;
        angles.thirdAngle *= -1;
    }

    // Autonomous Methods

    /**
     * Sets Drive to go forward/backwards
     * @param power Speed of movement
     */
    public void setDriveMotors(double power) {
        for (DcMotor i : driveMotors) {
            i.setPower(power);
        }
    }

    /**
     * Sets Drive to go left/right
     * @param power Speed of movement
     */
    public void setStrafeMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    /**
     * Sets Drive to Rotate
     * @param power Speed of Movement
     */
    public void setRotateMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    /**
     * Sets drive motors to move (forward/backwards) to a certain position
     * @param power Speed of movement
     * @param distance Distance to move (references to a target point)
     */
    public void encoderDrive(double power, int distance) {

        for (DcMotor i : driveMotors) {
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        for (DcMotor i : driveMotors) {
            i.setTargetPosition(distance);
        }
        for (DcMotor i : driveMotors) {
            i.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setDriveMotors(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy());

        setDriveMotors(0);
    }

    /**
     * Sets drive motors to move (left/right) to a certain position
     * @param power Speed of movement
     * @param distance Distance to move (references to a target point)
     */
    public void encoderStrafe(double power, int distance) {

        for (DcMotor i : driveMotors) {
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        frontLeft.setTargetPosition(distance);
        backLeft.setTargetPosition(-distance);
        frontRight.setTargetPosition(-distance);
        backRight.setTargetPosition(distance);

        for (DcMotor i : driveMotors) {
            i.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        setStrafeMotors(power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy());

        setDriveMotors(0);
    }

    /**
     * Sets drive motors to rotate to a certain angle - (-180, 180) range
     * @param power Speed of movement
     * @param heading rotation angle
     */
    public void rotateGyro(double power, double heading) {
        if (heading < 0) {
            while (angles.firstAngle < heading) {
                setRotateMotors(-power);
            }
            setDriveMotors(0);
        } else if (heading > 0) {
            while (angles.firstAngle > heading) {
                setRotateMotors(power);
            }
            setDriveMotors(0);
        }
    }

    // Driver-Controlled Methods

    /**
     * Tank Drive Control
     * @param left left side power
     * @param right right side power
     */
    public void tankDrive(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Rotational Force (GamePad Right Stick x)
     * @param z Left/Right (Strafe) Force (GamePad Left Stick x)
     */
    public void mecanumDrive(double y, double x, double z) {
        final double v1 = (y - x - z);
        final double v2 = (y - x + z);
        final double v3 = (y + x - z);
        final double v4 = (y + x + z);

        frontLeft.setPower(-3*v1/4);
        backLeft.setPower(-3*v2/4);
        backRight.setPower(-3*v3/4);
        frontRight.setPower(-3*v4/4);
    }

    /**
     * Mecanum Drive Control (Via trigonometric proportions)
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Rotational Force (GamePad Right Stick x)
     * @param z Left/Right (Strafe) Force (GamePad Left Stick x)
     */
    public void trigMecDrive(double y, double x, double z) {
        double r = Math.hypot(z, y);
        double robotangle = Math.atan2(y, z) - Math.PI/4;
        double rightX = x*-1;

        final double v1 = r * Math.cos(robotangle) + rightX;
        final double v2 = r * Math.sin(robotangle) - rightX;
        final double v3 = r * Math.sin(robotangle) + rightX;
        final double v4 = r * Math.cos(robotangle) - rightX;

        frontLeft.setPower(v1);
        backLeft.setPower(v2);
        backRight.setPower(v3);
        frontRight.setPower(v4);
    }

}
