package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "BLUECLOSE", group = "Autonomous")
//@Disabled
public class CLOSEBLUE extends LinearOpMode {
    MecanumDrive drive;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-53.25,-46.85, Math.toRadians(53));
        drive = new MecanumDrive(hardwareMap, beginPose);
        TrajectoryActionBuilder preload = drive.actionBuilder(beginPose)
                .waitSeconds(0.1)
                .lineToY(-24, new TranslationalVelConstraint(70), new ProfileAccelConstraint(-35, 70))
                .waitSeconds(1)
                .splineTo(new Vector2d(-12, -34.5), Math.toRadians(-90), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45,90))
                .splineTo(new Vector2d(-12, -62), Math.toRadians(-90))
                .waitSeconds(.1)
                .strafeTo(new Vector2d(-3, -59), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-35, 80))
                .strafeTo(new Vector2d(-3, -68), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-55, 90))
                .waitSeconds(0.6)
                .setTangent(90)
                .strafeToLinearHeading(new Vector2d(-16, -24), Math.toRadians(45), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45, 90))
                //.splineToLinearHeading(new Pose2d(-16,-24, Math.toRadians(50)), Math.toRadians(90), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-40, 80))
                .waitSeconds(1)
                .splineTo(new Vector2d(11, -37), Math.toRadians(-90), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45,90))
                .splineTo(new Vector2d(11, -67), Math.toRadians(-90))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-16, -24), Math.toRadians(45), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45, 90) )
                //.splineTo(new Vector2d(-16,-24), Math.toRadians(-140), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-40, 80))
                .waitSeconds(1)
                .splineTo(new Vector2d(33.5, -35), Math.toRadians(-90), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45,90))
                .splineTo(new Vector2d(33.5, -67), Math.toRadians(-90), new TranslationalVelConstraint(55), new ProfileAccelConstraint(-25,55))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-16, -24), Math.toRadians(45), new TranslationalVelConstraint(90), new ProfileAccelConstraint(-45, 90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-16, -50, Math.toRadians(0)), Math.toRadians(-90));
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                preload.build()
        );
    }
}