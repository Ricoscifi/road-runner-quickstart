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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name="Sid Solving Skill Issues")
public class SidSolvingSkillIssues extends LinearOpMode {

    public boolean oldCross;
    public boolean oldTriangle;
    public boolean oldCircle;
    public boolean oldSquare;
    public boolean oldRBumper;

    public boolean flywheel = false;
    public boolean primed = false;
    public boolean intaking = false;
    public boolean ejecting = false;

    public static double intakePower = 0.5;
    public static double transferPower = 0.5;
    public static double flywheelTargetRPM = -23.5;

    public DcMotorEx flywheel1;
    public DcMotorEx flywheel2;

    public DcMotor intake;

    public CRServo primeServo1;
    public CRServo primeServo2;

    private PIDController flywheelController;
    private final double p = 0.02, i = 0.00, d = 0.00045;
    public double f = 0.125;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit() && !isStopRequested()) {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel1 = hardwareMap.get(DcMotorEx.class, "leftflywheel");
            flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheel2 = hardwareMap.get(DcMotorEx.class, "rightflywheel");
            flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            flywheelController = new PIDController(p,i,d);


            primeServo1 = hardwareMap.get(CRServo.class, "leftPrimeServo");
            primeServo1.setDirection(DcMotorSimple.Direction.REVERSE);

            primeServo2 = hardwareMap.get(CRServo.class, "rightPrimeServo");
            primeServo1.setDirection(DcMotorSimple.Direction.REVERSE);

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
            } else if (gamepad1.right_bumper && !oldRBumper){
                flywheel1.setPower(0);
                flywheel2.setPower(0);
                flywheel = false;
            }
            oldRBumper = gamepad1.right_bumper;

            if (gamepad1.cross && !oldCross && !primed) {
                primeServo1.setPower(-transferPower);
                primeServo2.setPower(-transferPower);
                primed = true;
            } else if (gamepad1.cross && !oldCross && primed){
                primeServo1.setPower(0);
                primeServo2.setPower(0);
                flywheel = false;
            }
            oldCross = gamepad1.cross;

            if (gamepad1.square && !oldSquare && !intaking) {
                intake.setPower(intakePower);
                intaking = true;
            } else if (gamepad1.square && !oldSquare && intaking){
                intake.setPower(0);
                intaking = false;
            }
            oldSquare = gamepad1.square;

            if (gamepad1.circle && !oldCircle && !ejecting) {
                intake.setPower(-intakePower);
                ejecting = true;
            } else if (gamepad1.circle && !oldCircle && ejecting){
                intake.setPower(0);
                ejecting = false;
            }
            oldCircle = gamepad1.circle;

            telemetry.addData("Gamepad Right Bumper", gamepad1.right_bumper);
            telemetry.addData("flywheel:", flywheel);
            telemetry.addData("gamepad Cross", gamepad1.cross);
            telemetry.addData("Primed:", primed);
            telemetry.addData("OldCross", oldCross);
            telemetry.addData("Intaking", intaking);
            telemetry.update();

        }
    }
}
