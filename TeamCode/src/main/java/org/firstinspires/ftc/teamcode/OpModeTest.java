package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="OpModeTest")
public class OpModeTest extends LinearOpMode {

    public DcMotorEx[] driveMotors;

    public DcMotor flywheelleft;
    public DcMotor flywheelright;

    public DcMotor intake;

    public CRServo leftPrimeServo;
    public CRServo rightPrimeServo;

    public static double flywheelPower;
    public static double primeServoPower;
    public static double intakePower;

    double x = -gamepad1.left_stick_x;
    double y = -gamepad1.left_stick_y;
    double rx = gamepad1.right_stick_x;

    public boolean flywheel;
    public boolean primed;
    public boolean intaking;
    public boolean ejecting;

        public void runOpMode(){
            while (opModeInInit() && !isStopRequested()) {
              /*  driveMotors = new DcMotorEx[4];
                String[] motorNames = {"fL", "fR", "bL", "bR"};
                DcMotorSimple.Direction[] directions = {
                        DcMotorSimple.Direction.REVERSE,
                        DcMotorSimple.Direction.FORWARD,
                        DcMotorSimple.Direction.REVERSE,
                        DcMotorSimple.Direction.FORWARD
                };

                for (int i = 0; i < 4; i++) {
                    driveMotors[i] = hardwareMap.get(DcMotorEx.class, motorNames[i]);
                    driveMotors[i].setDirection(directions[i]);
                    driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

*/
                intake = hardwareMap.get(DcMotor.class, "intake");
                intake.setDirection(DcMotorSimple.Direction.FORWARD);
                intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                flywheelleft = hardwareMap.get(DcMotor.class, "leftflywheel");
                flywheelleft.setDirection(DcMotorSimple.Direction.FORWARD);
                flywheelleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                flywheelright = hardwareMap.get(DcMotor.class, "rightflywheel");
                flywheelright.setDirection(DcMotorSimple.Direction.FORWARD);
                flywheelright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                leftPrimeServo = hardwareMap.get(CRServo.class, "leftPrimeServo");
                rightPrimeServo = hardwareMap.get(CRServo.class, "rightPrimeServo");
            }
            while (opModeIsActive() && !isStopRequested()){

                if (gamepad1.cross && !flywheel){
                    flywheel = true;
                    flywheelleft.setPower(flywheelPower);
                    flywheelright.setPower(flywheelPower);
                } else if (gamepad1.cross && flywheel){
                    flywheel = false;
                    flywheelleft.setPower(0);
                    flywheelright.setPower(0);
                }

                    if (gamepad1.square && !primed){
                        primed = true;
                        leftPrimeServo.setPower(primeServoPower);
                        rightPrimeServo.setPower(-primeServoPower);
                    } else if (gamepad1.square && primed){
                        primed = false;
                        leftPrimeServo.setPower(0);
                        rightPrimeServo.setPower(0);
                }

                    if (gamepad1.circle && !intaking){
                        intaking = true;
                        intake.setPower(intakePower);
                    } else if (gamepad1.circle && intaking){
                        intaking = false;
                        intake.setPower(0);
                    }

                    if (gamepad1.triangle && !ejecting){
                        ejecting = true;
                        intake.setPower(-intakePower);
                    } else if (gamepad1.triangle && ejecting){
                        ejecting = false;
                        intake.setPower(0);
                    }

                    /*
                //Drive Motors
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
                // Send calculated power to wheels
                driveMotors[0].setPower(powerFrontLeft);
                driveMotors[1].setPower(powerFrontRight);
                driveMotors[2].setPower(powerBackLeft);
                driveMotors[3].setPower(powerBackRight);

                     */
                telemetry.addData("rightMotorPower:", flywheelPower);
                telemetry.addData("primed?", primed);
                telemetry.addData("flywheel", flywheel);
                telemetry.update();

            }


                }
            }



