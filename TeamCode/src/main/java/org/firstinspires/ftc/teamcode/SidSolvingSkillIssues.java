package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="Sid Solving Skill Issues")
public class SidSolvingSkillIssues extends LinearOpMode {

    public boolean oldCross;
    public boolean oldTriangle;
    public boolean oldCircle;
    public boolean oldSquare;
    public boolean oldRBumper;
    public boolean oldLBumper;

    public boolean flywheel = false;
    public boolean gate = false;
    public int intaking = 0;
    public boolean ejecting = false;

    public static double intakePower = 0.5;
    public static double transferPower = 0.5;
    public static double flywheelTargetRPM = -23.5;

    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;

    public DcMotor intake;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public Servo kicker;

    public CRServo transferServo1;
    public CRServo transferServo2;

    public Servo gateServo;
    public static double gateServoPos = 0;

    private PIDController flywheelController;
    private final double p = 0.02, i = 0.00, d = 0.00045;
    public double f = 0.125;

    @Override
    public void runOpMode() {
        while (opModeInInit() && !isStopRequested()) {

            frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
            backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
            backRight = hardwareMap.get(DcMotorEx.class, "backRight");

            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);


            kicker = hardwareMap.get(Servo.class, "kicker");
            kicker.setDirection(Servo.Direction.REVERSE);
            kicker.setPosition(1);

            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel1 = hardwareMap.get(DcMotorEx.class, "flyWheel1");
            flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel2 = hardwareMap.get(DcMotorEx.class, "flyWheel2");
            flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheelController = new PIDController(p,i,d);


            transferServo1 = hardwareMap.get(CRServo.class, "transferServo1");
            transferServo1.setDirection(DcMotorSimple.Direction.FORWARD);

            transferServo2 = hardwareMap.get(CRServo.class, "transferServo2");
            transferServo2.setDirection(DcMotorSimple.Direction.REVERSE);

            gateServo = hardwareMap.get(Servo.class, "gateServo");
            gateServo.setPosition(0.4);

          //  telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        while (opModeIsActive() && !isStopRequested()) {

            flywheelController.setPID(p,i,d);
            double speed = flywheel1.getVelocity()*60/28;

            double pow;
            if (Math.abs(speed) < 2 && flywheelTargetRPM == 0) {
                pow = 0;
            } else {
                pow = flywheelController.calculate(speed, flywheelTargetRPM);
            }

            if (gamepad1.right_bumper && !oldRBumper && !flywheel) {
                flywheel1.setPower(pow);
                flywheel2.setPower(pow);
                flywheel = true;

            } else if (gamepad1.right_bumper && !oldRBumper && flywheel){
                flywheel1.setPower(0);
                flywheel2.setPower(0);
                flywheel = false;
            }
            oldRBumper = gamepad1.right_bumper;

            if ( gamepad1.left_bumper && !oldLBumper && !gate){
                gateServo.setPosition(0);
                gate = true;
            } else if (gamepad1.left_bumper && !oldLBumper && gate){
                gateServo.setPosition(0.4);
                gate = false;
            }
            oldLBumper = gamepad1.left_bumper;
            if (gamepad1.square && !oldSquare){
                switch (intaking){
                    case 0:
                        intake.setPower(intakePower);
                        transferServo1.setPower(transferPower);
                        transferServo2.setPower(transferPower);
                        intaking = 2;
                        break;
                    case 1:
                        intake.setPower(0);
                        transferServo1.setPower(0);
                        transferServo2.setPower(0);
                        intaking = 0;
                        break;
                    case 2:
                        intake.setPower(intakePower - 0.05);
                        transferServo1.setPower(0);
                        transferServo2.setPower(0);
                        intaking = 1;
                        break;
                    case 3:
                        intake.setPower(0);
                        transferServo1.setPower(0);
                        transferServo2.setPower(0);
                        intaking = 0;
                }


            }
            oldSquare = gamepad1.square;

            if (gamepad1.triangle){
                intake.setPower(intakePower);
                transferServo1.setPower(-transferPower);
                transferServo2.setPower(-transferPower);
                intaking = 3;
            }
            if (gamepad1.circle && !oldCircle && !ejecting) {
                kicker.setPosition(0.85);
                ejecting = true;
            } else if (gamepad1.circle && !oldCircle && ejecting){
                kicker.setPosition(1);
                ejecting = false;
            }
            oldCircle = gamepad1.circle;

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double powerFrontLeft = y + x + rx;
            double powerFrontRight = y - x - rx;
            double powerBackLeft = (y - x + rx) * -1;
            double powerBackRight = (y + x - rx) * -1;

            if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                    Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
                // Find the largest power
                double max;
                max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
                max = Math.max(Math.abs(powerFrontRight), max);
                max = Math.max(Math.abs(powerBackRight), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                powerFrontLeft /= max;
                powerBackLeft /= max;
                powerFrontRight /= max;
                powerBackRight /= max;
            }

            frontLeft.setPower((float) powerFrontLeft);
            frontRight.setPower((float) powerFrontRight);
            backLeft.setPower(-(float) powerBackLeft);
            backRight.setPower(-(float) powerBackRight);



        telemetry.addData("Gamepad Right Bumper", gamepad1.right_bumper);
            telemetry.addData("flywheel:", flywheel);
            telemetry.addData("gamepad Cross", gamepad1.cross);
            telemetry.addData("Primed:", gate);
            telemetry.addData("OldCross", oldCross);
            telemetry.addData("Intaking", intaking);
            telemetry.update();

        }
    }
}
