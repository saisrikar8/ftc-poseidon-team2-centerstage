package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutoLeft")
public class AutoLeft extends LinearOpMode{
    public final int FLOOR = 0;
    public final int LOWPOLE = 1625;
    public final int MIDPOLE = 2725;
    public final int HIPOLE = 3860;
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory right1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(25)
                .build();
        Trajectory forward1 = drive.trajectoryBuilder(right1.end())
                .forward(51.5)
                .addDisplacementMarker(10, () -> {
                    drive.moveSlide(HIPOLE);
                    // This marker runs 20 inches into the trajectory

                    // Run your action in here!
                })
                .build();
        Trajectory left1 = drive.trajectoryBuilder(forward1.end())
                .strafeLeft(15)
                .build();
        Trajectory back1 = drive.trajectoryBuilder(left1.end())
                .back(4)
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(back1.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(33)
                .build();
        Trajectory right2 = drive.trajectoryBuilder(forward2.end())
                .strafeRight(3.5)
                .build();
        Trajectory back2 = drive.trajectoryBuilder(right2.end())
                .back(15.5)
                .build();
        Trajectory forward3 = drive.trajectoryBuilder(back2.end().plus(new Pose2d(0, 0, Math.toRadians(90))))
                .forward(1)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.closeClaw();
        sleep(1000);
        drive.moveSlide(200);
        drive.followTrajectory(right1);
        drive.followTrajectory(forward1);
        drive.followTrajectory(left1);
        drive.releaseClaw();
        drive.followTrajectory(back1);
        drive.turn(Math.toRadians(90));
        drive.moveSlide(600);
        drive.followTrajectory(forward2);
        drive.closeClaw();
        sleep(1000);
        drive.moveSlide(1625);
        drive.followTrajectory(right2);
        drive.followTrajectory(back2);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forward3);
        drive.releaseClaw();


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
