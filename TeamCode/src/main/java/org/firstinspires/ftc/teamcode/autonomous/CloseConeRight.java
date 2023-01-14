package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode .drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous(name = "CloseConeRight")
public class CloseConeRight extends LinearOpMode {
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


        Trajectory left1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(18)
                .build();
        Trajectory forward1 = drive.trajectoryBuilder(left1.end())
                .forward(10)
                .build();
        Trajectory right1 = drive.trajectoryBuilder(forward1.end())
                .strafeRight(12)
                .build();
        Trajectory forward2 = drive.trajectoryBuilder(right1.end())
                .forward(15)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.closeClaw();
        sleep(1000);
        drive.moveSlide(200);
        drive.followTrajectory(left1);
        sleep(1000);
        drive.moveSlide(LOWPOLE);
        sleep(1000);
        drive.followTrajectory(forward1);
        drive.releaseClaw();
        drive.followTrajectory(right1);
        drive.moveSlide(FLOOR);
        drive.followTrajectory(forward2);

        if(tagOfInterest.id == LEFT || tagOfInterest == null){
            Trajectory traj = drive.trajectoryBuilder(forward2.end())
                    .strafeLeft(15)
                    .build();
            drive.followTrajectory(traj);
        }else if(tagOfInterest.id == MIDDLE){
            Trajectory traj = drive.trajectoryBuilder(forward2.end())
                    .strafeRight(0)
                    .build();
            drive.followTrajectory(traj);
        }else {
            Trajectory traj = drive.trajectoryBuilder(forward2.end())
                    .strafeRight(15)
                    .build();
            drive.followTrajectory(traj);
        }


        while (!isStopRequested() && opModeIsActive()) ;
    }
}

