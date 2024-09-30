package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ChassisDrive")
public class ChassisDrive extends LinearOpMode {
    DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front-left");
    DcMotor rearLeft = hardwareMap.get(DcMotor.class, "rear-left");
    DcMotor frontRight = hardwareMap.get(DcMotor.class, "front-right");
    DcMotor rearRight = hardwareMap.get(DcMotor.class, "rear-right");
    public void runOpMode(){
        if (opModeInInit()){
            telemetry.addData("Status", "Initialized");
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            moveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

    }
    public void moveRobot(double leftStickX, double leftStickY, double rightStickX){
        double speed = leftStickY;
        double turn = rightStickX;
        double strafe = leftStickX;

        frontLeft.setPower(speed + turn + strafe);
        frontRight.setPower(speed-turn-strafe);
        rearLeft.setPower(speed +turn - strafe);
        rearRight.setPower(speed-turn+strafe);
    }
}
