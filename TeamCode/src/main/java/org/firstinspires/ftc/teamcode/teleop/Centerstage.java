package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "Centerstage")
public class Centerstage extends LinearOpMode {
    private SampleMecanumDrive drive;
    @Override
    public void runOpMode(){
        if (opModeInInit()) {
            drive = new SampleMecanumDrive(hardwareMap);
        }
        telemetry.addData("Status","Ready to Drive. Use right stick to turn, left stick to move forward/backward/strafe on game pad 1. Use game pad 1 right bumper to change to snail mode");
        telemetry.update();
        while (opModeIsActive()){
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.moveRobot(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
            if (gamepad2.a) {
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
        }
    }
}
