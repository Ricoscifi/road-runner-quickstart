package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@TeleOp(name="Position Lock with Pinpoint", group="TeleOp")
public class PositionLockTeleOp extends LinearOpMode {

    // Motors
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;


    private Pinpoint pinpoint;

    // PID Controllers for X, Y, and Heading
    private PIDController xPID, yPID, headingPID;


    // Target pose (position and heading)
    public static double targetX = 0;
    public static double targetY = 0;
    public static double targetHeading = 0;

    // Position lock state
    private boolean positionLocked = false;
    private boolean oldTriangle = false;

    // PID coefficients - tune these for your robot
    public static double Kp_X = 0.0;  // For X and Y
    public static double Ki_X = 0.0;
    public static double Kd_X = 0.00;

    public static double Kp_Y = 0.0;  // For X and Y
    public static double Ki_Y = 0.0;
    public static double Kd_Y = 0.0;

    public static double Kp_heading = 0.014;     // For heading/rotation
    public static double Ki_heading = 0.0;
    public static double Kd_heading = 0.00045;

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
        pinpoint = new Pinpoint(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));

        // Initialize PID controllers
        xPID = new PIDController(Kp_X, Ki_X, Kd_X);
        yPID = new PIDController(Kp_Y, Ki_Y, Kd_Y);
        headingPID = new PIDController(Kp_heading, Ki_heading, Kd_heading);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A = Toggle Position Lock");
        telemetry.addData("Controls", "Left Stick = Drive");
        telemetry.addData("Controls", "Right Stick X = Rotate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.triangle && !oldTriangle) {
                positionLocked = !positionLocked;
                if (positionLocked) {
                    // lockCurrentPosition(pos);
                }
            }

            oldTriangle = gamepad1.triangle;

            if (positionLocked) {
               // maintainPosition(pos);
            } else {
                normalDrive();
                // Reset PID controllers when not locked
                xPID.reset();
                yPID.reset();
                headingPID.reset();
            }

            // Telemetry
            telemetry.addData("Position Lock", positionLocked ? "ENABLED" : "DISABLED");
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
            telemetry.update();
        }
    }

    private void getCurrentPos() {
        targetX = pinpoint.getPosX();
        targetY = pinpoint.getPosY();
        targetHeading = pinpoint.getHeading();
        xPID.reset();
        yPID.reset();
        headingPID.reset();
    }

    private void maintainPosition() {
        // Get current position
        headingPID.setPID(Kp_heading,Ki_heading,Kd_heading);
        xPID.setPID(Kp_X, Ki_X, Kd_X);
        double headingPos = pinpoint.getHeading();
        double headingPow = headingPID.calculate(headingPos, targetHeading);
        double currentX = pinpoint.getPosX();
        double currentY = pinpoint.getPosY();

        // Calculate errors
        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        // Calculate PID outputs
        double correctionX = xPID.calculate(errorX);
        double correctionY = yPID.calculate(errorY);


        // Calculate motor powers using mecanum drive kinematics
        double leftFrontPower = -headingPow;
        double leftBackPower = -headingPow;
        double rightFrontPower = headingPow;
        double rightBackPower = headingPow;

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

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;


        double leftFrontPower = y + x + rx;
        double leftBackPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightBackPower = y + x - rx;


        double maxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Simple PID Controller class
   /* class PIDController {
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

    */
}