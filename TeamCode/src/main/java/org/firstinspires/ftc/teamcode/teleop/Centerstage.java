package org.firstinspires.ftc.teamcode.teleop;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Centerstage")
public class Centerstage extends LinearOpMode {

    private SampleMecanumDrive drive;
    public final int MAX_ROTATION = 140;
    public final double HIGH_TAPE = 24.75 * 2 / Math.sqrt(3);
    public final double MID_TAPE = 19 * 2 / Math.sqrt(3);
    public final double LOW_TAPE = 12.375 * 2 / Math.sqrt(3);
    public final double BACKDROP_LENGTH = ((double) (161)) / ((double) (8));

    @Override
    public void runOpMode() {
        if (opModeInInit()) {
            drive = new SampleMecanumDrive(hardwareMap);
        }
        telemetry.addData("Status", "Ready to Drive. Use right stick to turn, left stick to move forward/backward/strafe on game pad 1. Use game pad 1 right bumper to change to snail mode");
        telemetry.update();
        while (opModeIsActive()) {
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.moveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            drive.linearSlide.setPower(gamepad2.left_stick_y);
            drive.armRotation.setPower(gamepad2.right_stick_x);
            drive.setSlidePos();
            drive.setArmAngle();
            if (gamepad2.right_bumper) {
                telemetry.addData("Status", "Setting the pixel on the backdrop");
                if (drive.armAngle < 57.5 || drive.armAngle > 62.5) {
                    drive.rotateArm((-1 * drive.armAngle) + 60);
                }
                if (drive.slidePos < 24.5 || drive.slidePos > 25.5) {
                    drive.moveSlide((-1 * drive.slidePos) + 25);
                }
                drive.openClaw1();
                drive.openClaw2();
                sleep(30);
                drive.closeClaw1();
                drive.closeClaw2();
            }

            if (gamepad1.a) {
                drive.openClaw1();
                telemetry.addData("Claw 1 Status", "Opened Claw 1");
            }
            if (gamepad1.x) {
                drive.closeClaw1();
                telemetry.addData("Claw 1 Status", "Closed Claw 2");
            }
            if (gamepad1.b) {
                drive.openClaw2();
                telemetry.addData("Claw 2 Status", "Opened Claw 2");
            }
            if (gamepad1.y) {
                drive.closeClaw2();
                telemetry.addData("Claw 2 Status", "Closed Claw 2");
            }


            if (gamepad2.dpad_up) {
                drive.rotateArm(90 - drive.armAngle);
            }
            if (gamepad2.dpad_left) {
                drive.rotateArm(0 - drive.armAngle);
            }
            if (gamepad2.dpad_right) {
                drive.rotateArm(MAX_ROTATION - drive.armAngle);
            }
            if (gamepad2.a) {
                drive.moveSlide(HIGH_TAPE - drive.slidePos);
            }
            if (gamepad2.y) {
                drive.moveSlide(MID_TAPE - drive.slidePos);
            }
            if (gamepad2.x) {
                drive.moveSlide(LOW_TAPE - drive.slidePos);
            }
            if (gamepad2.b) {
                drive.moveSlide(-drive.slidePos);
            }
            if (gamepad2.left_bumper) {
                telemetry.addData("Status", "Searching for April Tag");
                drive.rotateArm((MAX_ROTATION - 30) - drive.armAngle);
                AprilTagProcessor scanner = new AprilTagProcessor.Builder()
                        .setDrawCubeProjection(true)
                        .setDrawTagID(true)
                        .build();
                VisionPortal visionPortal = new VisionPortal.Builder()
                        .addProcessor(scanner)
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .setCameraResolution(new Size(640, 480))
                        .build();
                List<AprilTagDetection> detections = scanner.getDetections();
                if (detections.size() > 0 && detections.get(0).id <= 6) {
                    AprilTagDetection detection = detections.get(0);
                    AprilTagPoseFtc detectionPosition = detections.get(0).ftcPose;
                    telemetry.addData("Status", "April Tag Found. Adjusting Robot Position.");
                    //adjusting bearing of robot to minimize error
                    int iterations = 0;
                    while (detectionPosition.bearing > 0.3 || iterations < 3000) {
                        drive.turn(-detectionPosition.bearing);
                        detection = scanner.getDetections().get(0);
                        detectionPosition = detection.ftcPose;
                        iterations++;
                    }
                    iterations = 0;
                    //adjusting distance of robot from backdrop to minimize error
                    while (detectionPosition.y > 3 || iterations < 3000) {
                        drive.moveRobot((detectionPosition.y - 4) / 20, 0, 0, false);
                        detection = scanner.getDetections().get(0);
                        detectionPosition = detection.ftcPose;
                        iterations++;
                    }
                    iterations = 0;
                    //adjusting alignment of robot to correct backdrop positioning
                    while (detectionPosition.x > 1 || iterations < 3000) {
                        drive.moveRobot(0, (detectionPosition.x - 4) / 20, 0, false);
                        detection = scanner.getDetections().get(0);
                        detectionPosition = detection.ftcPose;
                        iterations++;
                    }
                    drive.rotateArm(120 - drive.armAngle);
                    double desiredSlidePos = (detectionPosition.y / BACKDROP_LENGTH) * HIGH_TAPE;
                    drive.moveSlide(desiredSlidePos - drive.slidePos);
                } else {
                    telemetry.addData("Status", "You are not near the backdrop. Maybe you need to turn around.");
                }
            }
        }
    }
}
