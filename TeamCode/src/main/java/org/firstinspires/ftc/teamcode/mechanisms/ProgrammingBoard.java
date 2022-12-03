package org.firstinspires.ftc.teamcode.mechanisms;

/**
 *Imports physical hardware to manipulate
 */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ProgrammingBoard {
    /**
     * These motors are the 4 mecanum wheels that drive the robot
     */
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    public Servo claw;
    private DcMotor linearSlide;
    public double nitro;
    public double speed;

    /**
     * Define slide positions
     */
    public final int FLOOR = 0;
    public final int LOWPOLE = 1625;
    public final int MIDPOLE = 2725;
    public final int HIPOLE = 3860;
    /*
    public final int FLOOR = 150;
    public final int LOWPOLE = 1950;
    public final int MIDPOLE = 3100;
    public final int HIPOLE = 4300;
     */

    // public final int LOWPOLE = 1000;
    // public final int MIDPOLE = 2100;
    // public final int HIPOLE = 3000;
    public final double TICKS_PER_REV = 527.62;
    /**
     * Power of linear slide
     */
    double slidePower = 0.5;
    // double drive;  //  Power for forward and back motion
    // double strafe;  // Power for left and right motion
    // double rotate;  // Power for rotating the robot

    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        claw = hwMap.get(Servo.class, "claw");
        linearSlide = hwMap.get(DcMotor.class, "linearSlide");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nitro = 2.0;
        speed = 1;

    }


    public void moveRobot(double leftStickY, double leftStickX, double rightStickX){
        /**
         * Wheel powers calculated using gamepad 1's inputs leftStickY, leftStickX, and rightStickX
         * **/
        double frontLeftPower = leftStickY + leftStickX - rightStickX;
        double backLeftPower = -leftStickY + leftStickX + rightStickX;
        double frontRightPower = leftStickY - leftStickX + rightStickX;
        double backRightPower = -leftStickY - leftStickX - rightStickX;

        /**
         * Sets the wheel's power
         * **/
        nitro = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1) * speed;
        frontLeft.setPower(frontLeftPower/speed);
        backLeft.setPower(backLeftPower/speed);
        frontRight.setPower(frontRightPower/speed);
        backRight.setPower(backRightPower/speed);
    }

    public void releaseClaw() {

        claw.setPosition(0.4);
    }
    public void closeClaw() {
        claw.setPosition(0.65);
    }
    public double getPosition() {
        return claw.getPosition();
    }
    public void moveClaw(double d) {
        claw.setPosition(d);
    }

    /**
     * Method to move slide to a given stage
     * @param slidePos
     */
    public void moveSlide(int slidePos) {
        linearSlide.setTargetPosition(slidePos);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(slidePower);
    }
    /**
     * This autonomous strafe method allows the robot to strafe using mecanum wheels and the direction
     * and the direction is specified via a boolean variable
     * **/
    public void autonomousMotorStrafe(boolean topLeft, boolean bottomLeft,boolean topRight,boolean bottomRight){
        if(topLeft){
            frontLeft.setPower( 0);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower( 0);
        }
        else if(bottomLeft){
            frontLeft.setPower(-0.5);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(-0.5);
        }
        else if(topRight){
            frontLeft.setPower(0.5);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0.5);
        }
        else if(bottomRight){
            frontLeft.setPower( 0);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower( 0);
        }
    }

}