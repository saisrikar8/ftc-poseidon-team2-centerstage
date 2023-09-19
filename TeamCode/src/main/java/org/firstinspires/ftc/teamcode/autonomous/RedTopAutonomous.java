package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RedTopAutonomous")
public class RedTopAutonomous extends LinearOpMode {
    private SampleMecanumDrive drive;

    private DcMotor rearLeft;
    private DcMotor rearRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor armRotationMotor1;
    private DcMotor armRotationMotor2;

    private DcMotor armExtendingMotor1;
    private DcMotor armExtendingMotor2;

    private DistanceSensor propSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMotorPowers(1, 1, 1, 1);

        rearLeft = hardwareMap.dcMotor.get("rear-left");
        rearRight = hardwareMap.dcMotor.get("rear-right");
        frontLeft = hardwareMap.dcMotor.get("front-left");
        frontRight = hardwareMap.dcMotor.get("front-right");

        armRotationMotor1 = hardwareMap.dcMotor.get("arm-rotation-1");
        armRotationMotor2 = hardwareMap.dcMotor.get("arm-rotation-2");

        armExtendingMotor1 = hardwareMap.dcMotor.get("arm-extending-1");
        armExtendingMotor2 = hardwareMap.dcMotor.get("arm-extending-2");

        propSensor = hardwareMap.get(DistanceSensor.class, "prop-sensor");

        waitForStart();

        /*
         This while loop will run once the RUN button is clicked on the FTC App.
         */
        int propLocation;
        double propDistance;
        Pose2d firstEndPosition;
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(70, 10, Math.toRadians(180)))
                .forward(30)
                .build();
        drive.followTrajectory(traj1);
        drive.turn(-1*Math.PI);
        firstEndPosition = new Pose2d(traj1.end().getX(), traj1.end().getY(),0);
        propDistance = getDistanceFromSensor(propSensor);
        if (propDistance > 7 && propDistance < 9){
            propLocation = RedTapeMark.CENTER.getValue();
            drive.moveSlide(8);
            drive.releaseClaw();
        }
        else{
            drive.turn(Math.toRadians(-90));
            Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                    .strafeLeft(10)
                    .build();
            drive.followTrajectory(traj2);
            propDistance = getDistanceFromSensor(propSensor);
            drive.moveSlide(4);
            if (propDistance > 1 && propDistance < 3) {
                propLocation = RedTapeMark.RIGHT.getValue();
                drive.releaseClaw();
            }
            else{
                propLocation = RedTapeMark.LEFT.getValue();
                drive.rotateArm(0.5);
                drive.releaseClaw();
            }
            firstEndPosition = new Pose2d(traj2.end().getX(), traj2.end().getY(), Math.toRadians(90));
        }
        Trajectory traj3 = drive.trajectoryBuilder(firstEndPosition)
                .splineTo(new Vector2d(60, 40), Math.toRadians(90))
                .splineTo(new Vector2d(40, 40), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj3);
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft((propLocation - 3)*5)
                .strafeLeft((6-propLocation)*5 + 15)
                .build();
        drive.followTrajectory(traj4);
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(11, -33), Math.toRadians(270))
                .splineTo(new Vector2d(-24, -57.5), Math.toRadians(180))
                .build();
        drive.followTrajectory(traj5);
    }




        //fetches distance of prop from sensor when needed
    public double getDistanceFromSensor(DistanceSensor sensor){
        double distance = sensor.getDistance(DistanceUnit.INCH);
        return distance;
    }
}
