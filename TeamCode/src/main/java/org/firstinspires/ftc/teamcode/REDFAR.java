package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "REDFAR", group = "Autonomous")
//@Disabled
public class REDFAR extends LinearOpMode {
    MecanumDrive drive;
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-25,-25,Math.toRadians(45));
        drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder preload = drive.actionBuilder(beginPose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-12.5, -25, Math.toRadians(270)), Math.toRadians(180))
                .waitSeconds(0.1)
                .lineToY(-50)
                .splineToLinearHeading(new Pose2d(-25,-25, Math.toRadians(45)), Math.toRadians(45))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(12.5,-25,Math.toRadians(270)), Math.toRadians(0))
                .waitSeconds(0.1)
                .lineToY(-50)
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(-25,-25, Math.toRadians(45)), Math.toRadians(180))
                .waitSeconds(1);
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                preload.build()
        );
    }
}