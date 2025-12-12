package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name="OpModeTest")
public class OpModeTest extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    public DcMotorEx flywheelleft;
    public DcMotorEx flywheelright;

    public DcMotor intake;

    public CRServo leftPrimeServo;
    public CRServo rightPrimeServo;

    public static double primeServoPower = 0;
    public static double intakePower = 0;

    //double x = -gamepad1.left_stick_x;
    //double y = -gamepad1.left_stick_y;
    //double rx = gamepad1.right_stick_x;

    public boolean oldCross;
    public boolean oldTriangle;
    public boolean oldCircle;
    public boolean oldSquare;

    public boolean flywheel = false;
    public boolean primed = false;
    public boolean intaking = false;
    public boolean ejecting = false;

    private PIDController flywheelPID;
    private final double p = 0.02, i = 0.00, d = 0.00045;
    public double f = 0.125;
    public static int targetRPM = 0;

    public void runOpMode() {
        while (opModeInInit() && !isStopRequested()) {

            //leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            //leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            //rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            //rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

           // leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           // leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           // rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
           // rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheelleft = hardwareMap.get(DcMotorEx.class, "leftflywheel");
            flywheelleft.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheelleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheelright = hardwareMap.get(DcMotorEx.class, "rightflywheel");
            flywheelright.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheelright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheelPID = new PIDController(p,i,d);


            leftPrimeServo = hardwareMap.get(CRServo.class, "leftPrimeServo");
            rightPrimeServo = hardwareMap.get(CRServo.class, "rightPrimeServo");

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        while (opModeIsActive() && !isStopRequested()) {
            setFlyWheelPower();

            if (gamepad1.cross && !oldCross) {
                if (!flywheel) {
                    targetRPM = 13;
                    flywheel = true;
                } else {
                    targetRPM = 0;
                    flywheel = false;
                }
            }
                oldCross = gamepad1.cross;

                if (gamepad1.square && !oldSquare) {
                    if (!primed){
                        leftPrimeServo.setPower(primeServoPower);
                        rightPrimeServo.setPower(-primeServoPower);
                        primed = true;
                    } else {
                        leftPrimeServo.setPower(0);
                        rightPrimeServo.setPower(0);
                        primed = false;
                    }
                }
                oldSquare = gamepad1.square;

                if (gamepad1.circle && !oldCircle) {
                    if (!intaking){
                        intake.setPower(intakePower);
                        intaking = true;
                    } else {
                        intake.setPower(0);
                        intaking = false;
                    }
                }
                oldCircle = gamepad1.circle;

                if (gamepad1.triangle && !oldTriangle) {
                    if (!ejecting){
                        intake.setPower(-intakePower);
                        ejecting = true;
                    } else {
                        intake.setPower(0);
                        ejecting = false;
                    }
                }
                oldTriangle = gamepad1.triangle;


               /* double powerFrontLeft = y + x + rx;
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
                // Send calculated power to wheels
                leftFront.setPower(powerFrontLeft);
                rightFront.setPower(powerFrontRight);
                leftBack.setPower(powerBackLeft);
                rightBack.setPower(powerBackRight);

                */

                telemetry.addData("gate?", primed);
                telemetry.addData("flywheel", flywheel);
                telemetry.addData("target ", targetRPM);
                telemetry.update();
            }
        }

        public void setFlyWheelPower(){
            flywheelPID.setPID(p, i, d);
            int flywheelRightCurrPos = flywheelright.getCurrentPosition();
            telemetry.addData("pos ", flywheelRightCurrPos);
            double shooter1Speed = flywheelPID.calculate(flywheelright.getVelocity(AngleUnit.RADIANS)/2 *30/Math.PI, targetRPM) + f;
            telemetry.addData("pow ", shooter1Speed);

            flywheelright.setPower(-shooter1Speed);
            flywheelleft.setPower(-shooter1Speed);
        }
    }




