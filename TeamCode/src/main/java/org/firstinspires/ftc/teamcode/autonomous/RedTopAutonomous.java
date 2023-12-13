package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


@Autonomous(name = "RedTopAutonomous")
public class RedTopAutonomous extends LinearOpMode {
    private SampleMecanumDrive drive;
    private int propLocation;
    private DistanceSensor propSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMotorPowers(1, 1, 1, 1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

        /*
         This while loop will run once the RUN button is clicked on the FTC App.
         */
            Pose2d firstEndPosition;

            Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(70, 10, Math.toRadians(180)))
                    .forward(30)
                    .build();
            drive.followTrajectory(traj1);
            drive.turn(-1 * Math.PI);
            firstEndPosition = new Pose2d(traj1.end().getX(), traj1.end().getY(), 0);
            if (detectProp()) {
                propLocation = RedTapeMark.CENTER.getValue();
                drive.moveSlide(8);
                drive.openClaw1();
            } else {
                drive.turn(Math.toRadians(-90));
                Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                        .strafeLeft(10)
                        .build();
                drive.followTrajectory(traj2);
                drive.moveSlide(4);
                if (detectProp()) {
                    propLocation = RedTapeMark.RIGHT.getValue();
                    drive.openClaw1();
                    sleep(100);
                    drive.closeClaw1();
                } else {
                    propLocation = RedTapeMark.LEFT.getValue();
                    drive.rotateArm(0.5);
                    drive.openClaw1();
                    sleep(100);
                    drive.closeClaw1();
                }
                firstEndPosition = new Pose2d(traj2.end().getX(), traj2.end().getY(), Math.toRadians(90));
            }
            Trajectory traj3 = drive.trajectoryBuilder(firstEndPosition)
                    .splineTo(new Vector2d(60, 40), Math.toRadians(90))
                    .splineTo(new Vector2d(40, 40), Math.toRadians(90))
                    .build();
            drive.followTrajectory(traj3);
            Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                    .strafeLeft((propLocation - 3) * 5)
                    .strafeLeft((6 - propLocation) * 5 + 15)
                    .build();
            drive.followTrajectory(traj4);
            Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                    .splineTo(new Vector2d(11, -33), Math.toRadians(270))
                    .splineTo(new Vector2d(-24, -57.5), Math.toRadians(180))
                    .build();
            drive.followTrajectory(traj5);
        }
    }

    public boolean detectProp(){
        TfodProcessor objectDetector = TfodProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), objectDetector);
        if (objectDetector.getRecognitions().size() > 0 && objectDetector.getRecognitions().get(0).getConfidence() >= 0.9){
            return true;
        }
        return false;
    }
        //evaluates whether the point on the backdrop is correct using the april tag
    public boolean determineIsCorrectPositionUsingPhoneCam(){
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        //getting details of April tag
        AprilTagDetection detection = tagProcessor.getDetections().get(0);
        int tagId = detection.id;
        if (propLocation == tagId) {
            AprilTagPoseFtc detectionPosition = detection.ftcPose;

            //adjusting bearing of robot to minimize error
            while (detectionPosition.bearing > 0.3) {
                drive.turn(-detectionPosition.bearing);
                detection = tagProcessor.getDetections().get(0);
                detectionPosition = detection.ftcPose;
            }
            //adjusting distance of robot from backdrop to minimize error
            while (detectionPosition.y > 5){
                drive.moveRobot((detectionPosition.y - 4)/20, 0, 0, false);
                detection = tagProcessor.getDetections().get(0);
                detectionPosition = detection.ftcPose;
            }
            //adjusting alignment of robot to correct backdrop positioning
            while(detectionPosition.x > 1){
                drive.moveRobot(0, (detectionPosition.x - 4)/20, 0, false);
                detection = tagProcessor.getDetections().get(0);
                detectionPosition = detection.ftcPose;
            }
            return true;
        }
        return false;
    }
}
