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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "AutoLeft")
public class AutoLeft extends LinearOpMode{
    public final int FLOOR = 0;
    public final int LOWPOLE = 1625;
    public final int MIDPOLE = 2725;
    public final int HIPOLE = 3860;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }


            }
        }

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
                .forward(29.0)
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
        drive.moveSlide(LOWPOLE);
        drive.followTrajectory(right2);
        drive.followTrajectory(back2);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(forward3);
        drive.releaseClaw();
        if(tagOfInterest.id == LEFT || tagOfInterest == null){
            Trajectory traj = drive.trajectoryBuilder(forward3.end())
                    .strafeRight(16)
                    .build();
            drive.followTrajectory(traj);
        }else if(tagOfInterest.id == MIDDLE){
            Trajectory traj = drive.trajectoryBuilder(forward3.end())
                    .strafeLeft(12)
                    .build();
            drive.followTrajectory(traj);
        }else {
            Trajectory traj = drive.trajectoryBuilder(forward3.end())
                    .strafeLeft(43)
                    .build();
            drive.followTrajectory(traj);
        }
        drive.moveSlide(FLOOR);


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
