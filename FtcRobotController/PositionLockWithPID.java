package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Import your Pinpoint driver
// import com.example.GoBildaPinpointDriver;

@TeleOp(name="Position Lock with Pinpoint", group="TeleOp")
public class PositionLockTeleOp extends LinearOpMode {

    // Motors
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    // Pinpoint Odometry Computer
    // private GoBildaPinpointDriver odo;

    // PID Controllers for X, Y, and Heading
    private PIDController xPID, yPID, headingPID;

    // Target pose (position and heading)
    private double targetX = 0;
    private double targetY = 0;
    private double targetHeading = 0;

    // Position lock state
    private boolean positionLocked = false;
    private boolean wasAPressed = false;

    // PID coefficients - tune these for your robot
    private static final double Kp_translation = 0.08;  // For X and Y
    private static final double Ki_translation = 0.0;
    private static final double Kd_translation = 0.005;

    private static final double Kp_heading = 0.02;     // For heading/rotation
    private static final double Ki_heading = 0.0;
    private static final double Kd_heading = 0.002;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set motor directions (adjust based on your robot configuration)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior to BRAKE for better resistance
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Pinpoint Odometry
        // odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        // odo.setOffsets(-84.0, -168.0); // Set your offsets in mm
        // odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
        //                          GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // odo.resetPosAndIMU();

        // Initialize PID controllers
        xPID = new PIDController(Kp_translation, Ki_translation, Kd_translation);
        yPID = new PIDController(Kp_translation, Ki_translation, Kd_translation);
        headingPID = new PIDController(Kp_heading, Ki_heading, Kd_heading);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A = Toggle Position Lock");
        telemetry.addData("Controls", "Left Stick = Drive");
        telemetry.addData("Controls", "Right Stick X = Rotate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            // odo.update();
            // Pose2D pos = odo.getPosition();

            // For testing without actual Pinpoint hardware, use dummy values
            Pose2D pos = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);

            // Toggle position lock with A button
            if (gamepad1.a && !wasAPressed) {
                positionLocked = !positionLocked;
                if (positionLocked) {
                    // Set current position as target
                    lockCurrentPosition(pos);
                }
            }
            wasAPressed = gamepad1.a;

            if (positionLocked) {
                // Apply PID control to maintain position
                maintainPosition(pos);
            } else {
                // Normal mecanum drive
                normalDrive();
                // Reset PID controllers when not locked
                xPID.reset();
                yPID.reset();
                headingPID.reset();
            }

            // Telemetry
            telemetry.addData("Position Lock", positionLocked ? "ENABLED" : "DISABLED");
            telemetry.addData("Current X (mm)", pos.getX(DistanceUnit.MM));
            telemetry.addData("Current Y (mm)", pos.getY(DistanceUnit.MM));
            telemetry.addData("Current Heading (deg)", Math.toDegrees(pos.getHeading(AngleUnit.RADIANS)));

            if (positionLocked) {
                telemetry.addData("Target X (mm)", targetX);
                telemetry.addData("Target Y (mm)", targetY);
                telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
                telemetry.addData("X Error (mm)", targetX - pos.getX(DistanceUnit.MM));
                telemetry.addData("Y Error (mm)", targetY - pos.getY(DistanceUnit.MM));
            }

            telemetry.update();
        }
    }

    private void lockCurrentPosition(Pose2D currentPos) {
        targetX = currentPos.getX(DistanceUnit.MM);
        targetY = currentPos.getY(DistanceUnit.MM);
        targetHeading = currentPos.getHeading(AngleUnit.RADIANS);

        // Reset PID controllers
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }

    private void maintainPosition(Pose2D currentPos) {
        // Get current position
        double currentX = currentPos.getX(DistanceUnit.MM);
        double currentY = currentPos.getY(DistanceUnit.MM);
        double currentHeading = currentPos.getHeading(AngleUnit.RADIANS);

        // Calculate errors
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorHeading = normalizeAngle(targetHeading - currentHeading);

        // Calculate PID outputs
        double correctionX = xPID.calculate(errorX);
        double correctionY = yPID.calculate(errorY);
        double correctionHeading = headingPID.calculate(errorHeading);

        // Convert field-centric corrections to robot-centric
        double robotX = correctionX * Math.cos(-currentHeading) - correctionY * Math.sin(-currentHeading);
        double robotY = correctionX * Math.sin(-currentHeading) + correctionY * Math.cos(-currentHeading);

        // Calculate motor powers using mecanum drive kinematics
        double leftFrontPower = robotY + robotX + correctionHeading;
        double leftBackPower = robotY - robotX + correctionHeading;
        double rightFrontPower = robotY - robotX - correctionHeading;
        double rightBackPower = robotY + robotX - correctionHeading;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    private void normalDrive() {
        // Standard mecanum drive with gamepad
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe
        double rx = gamepad1.right_stick_x; // Rotation

        // Calculate motor powers
        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;

        // Normalize powers
        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    // Normalize angle to [-pi, pi]
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Simple PID Controller class
    class PIDController {
        private double Kp, Ki, Kd;
        private double integral = 0;
        private double lastError = 0;
        private long lastTime = System.nanoTime();

        public PIDController(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }

        public double calculate(double error) {
            long currentTime = System.nanoTime();
            double dt = (currentTime - lastTime) / 1e9; // Convert to seconds

            // Integral with anti-windup
            integral += error * dt;
            if (Math.abs(integral) > 1000) {
                integral = Math.signum(integral) * 1000;
            }

            // Derivative
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            // PID output
            double output = Kp * error + Ki * integral + Kd * derivative;

            // Update for next iteration
            lastError = error;
            lastTime = currentTime;

            return output;
        }

        public void reset() {
            integral = 0;
            lastError = 0;
            lastTime = System.nanoTime();
        }
    }
}