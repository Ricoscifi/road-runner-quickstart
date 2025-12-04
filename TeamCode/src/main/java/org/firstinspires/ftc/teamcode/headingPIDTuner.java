
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Pinpoint;

@Config
@TeleOp
public class headingPIDTuner extends OpMode {
    private PIDController controller1;
    private PIDController controller2;

    public static double p = 0.05, i = 0.00, d = 0.001, f = 0.0;
    public static double e = 2;
    public static double targetX = 0;
    private final double angleSlides = 60;

    public static double yp = 0, yi = 0, yd = 0;
    public static double targetY = 0;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private VoltageSensor voltageSensor;
    private Pinpoint pinpoint;


    @Override
    public void init() {
        controller1 = new PIDController(p, i, d);
        controller2 = new PIDController(yp, yi, yd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint = new Pinpoint(hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"));
        pinpoint.resetIMU();
    }

    @Override
    public void loop() {

            controller1.setPID(p, i, d);
            double pos = pinpoint.getPosX();
            double pow = controller1.calculate(pos, targetX);
            if (pos - targetX > e) {
                pow += f;
                telemetry.addData("sub", "s");
            } else if (targetX - pos < -e) {
                pow += f;
                telemetry.addData("add", "a");
            }
            telemetry.addData("pow ", pow);
            telemetry.addData("pos ", pos);

            double maxPower = Math.abs(pow);
            if (maxPower > 1.0) {
                pow /= maxPower;
            }

            controller2.setPID(yp, yi, yd);
            double ypos = pinpoint.getPosY();
            double ypow = controller2.calculate(ypos, targetY);


            leftFront.setPower(-ypow + pow);
            leftBack.setPower(ypow + pow);
            rightFront.setPower(ypow + pow);
            rightBack.setPower(-ypow + pow);

            telemetry.addData("target ", targetX);
            telemetry.addData("YTarget", targetY);
            telemetry.addData("Y Pos", ypos);
            telemetry.addData("Y pow", ypow);
            telemetry.addData("X Pos", pos);
            telemetry.update();
        }
    }
