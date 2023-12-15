package org.firstinspires.ftc.FinalBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class MHardware {
    //Motors and Servos
    private LinearOpMode finalOpMode = null;
    private DcMotor leftFrontDrive = null; //GoBUILDA
    private DcMotor leftBackDrive = null; //GoBUILDA
    private DcMotor rightFrontDrive = null; //GoBUILDA
    private DcMotor rightBackDrive = null; //GoBUILDA
    private DcMotor hangMotor = null; //Tetrix TorqueNADO
    private DcMotor liftIntakeMotor = null; //GoBUILDA
    private Servo intakeServo = null; //Linear Actuator for open/close intake
    private Servo angleIntakeLiftServo = null; //Linear Actuator for angling lift
    private Servo angleIntakeServo = null; //Servo to angle the intake
    private Servo angleDroneServo = null; //Servo
    private Servo launchDroneServo = null; //Servo

    //Constants
    public static final double GRAB_PIXEL = 0;
    public static final double RELEASE_PIXEL = 45;
    public static final double ANGLE_DRONE = 45;
    public static final double ANGLE_DRONE_RESTPOS = 90;
    public static final double LAUNCH_DRONE = 90;
    public static final double INTAKELIFT_BACKPOS = 0;
    public static final double INTAKELIFT_STRAIGHTPOS = 45;
    public static final double INTAKELIFT_FORWARDPOS = 90;
    public static final double INTAKE_COLLECTPOS = 90;
    public static final double INTAKE_RESTPOS = 0;
    public static final double LIFT_INTAKE = 0.8;
    public static final double HANG_ROBOT = 0.5;
    public static final double LIFT_COUNTSPERINCH = 0;

    public MHardware(LinearOpMode opmode) {
        finalOpMode = opmode;
    }

    public void init() {
        //Assigning the variables made earlier to the motors on the hardware configuration and setting their directions
        leftFrontDrive = finalOpMode.hardwareMap.get(DcMotor.class, "leftf_drive");
        leftBackDrive = finalOpMode.hardwareMap.get(DcMotor.class, "leftb_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive = finalOpMode.hardwareMap.get(DcMotor.class, "rightf_drive");
        rightBackDrive = finalOpMode.hardwareMap.get(DcMotor.class, "rightb_drive");
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Same down here + setting 2 motors to run with encoders for moving to a position with ease
        hangMotor = finalOpMode.hardwareMap.get(DcMotor.class, "hangMotor");
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftIntakeMotor = finalOpMode.hardwareMap.get(DcMotor.class, "liftIntakeMotor");
        liftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeServo = finalOpMode.hardwareMap.get(Servo.class, "intakeServo");
        angleIntakeLiftServo = finalOpMode.hardwareMap.get(Servo.class, "angleIntakeLiftServo");
        angleIntakeServo = finalOpMode.hardwareMap.get(Servo.class, "angleIntakeServo");
        angleDroneServo = finalOpMode.hardwareMap.get(Servo.class, "angleDroneServo");
        launchDroneServo = finalOpMode.hardwareMap.get(Servo.class, "launchDroneServo");

        //Letting the users know that the robot is fully initialized
        finalOpMode.telemetry.addData("Status", "Robot Ready");
        finalOpMode.telemetry.update();


    }

    //To set the powers for the Mecanum wheels
    public void setPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(leftBackPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(rightBackPower));

        if (maxSpeed > 1.0) {
            leftFrontPower /= maxSpeed;
            leftBackPower /= maxSpeed;
            rightFrontPower /= maxSpeed;
            rightBackPower /= maxSpeed;
        }

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    //TO DRIVE ROBOT (for Mecanum wheels)
    public void driveRobot(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        setPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }

    //HANGING MECHANISM TO LIFT ROBOT (motor)
    public void hangRobot(double hang) {
        setHangPower(hang);
    }

    public void setHangPower(double hangInput) {
        hangMotor.setPower(hangInput);
    }

    //LIFT THE INTAKE  (motor)
//    public void liftIntakeEncoder(double speed, double liftMotorInches, double timeoutS) {
//        int newLiftMotorTarget;
//        newLiftMotorTarget = liftIntakeMotor.getCurrentPosition() + (int)(liftMotorInches * LIFT_COUNTSPERINCH);
//        liftIntakeMotor.setPower(lift);
//    } //^UNDER CONSTRUCTION TO MOVE LIFT USING ENCODER COUNTS^

    //OPEN/CLOSE THE INTAKE (servo)
    public void grabPixel(double grab) {
        intakeServo.setPosition(grab);
    }

    public void releasePixel(double release) {
        intakeServo.setPosition(release);
    }

    //ANGLE THE DRONE LAUNCHER (servo)
    public void angleDrone(double angleDrone) {
        angleDroneServo.setPosition(angleDrone);
    }

    //SHOOT DRONE(servo)
    public void launchDrone(double launch) {
        launchDroneServo.setPosition(launch);
    }

    //ANGLE THE LIFT FOR THE INTAKE (linear actuator)
    public void posAngleBack(double angleBack) {
        angleIntakeLiftServo.setPosition(angleBack);
    }

    public void posAngleStraight(double angleStraight) {
        angleIntakeLiftServo.setPosition(angleStraight);
    }

    public void posAngleForward(double angleForward) {
        angleIntakeLiftServo.setPosition(angleForward);
    }

    //ANGLE THE INTAKE (linear actuator)
    public void intakeCollectPos(double collect) {
        angleIntakeServo.setPosition(collect);
    }

    public void intakeRestPos(double rest) {
        angleIntakeServo.setPosition(rest);
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double leftRearInches, double rightFrontInches, double rightRearInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftFrontTarget = leftFront.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
//            newLeftRearTarget = leftRear.getCurrentPosition() + (int)(leftRearInches * COUNTS_PER_INCH);
//            newRightFrontTarget = rightFront.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
//            newRightRearTarget = rightRear.getCurrentPosition() + (int)(rightRearInches * COUNTS_PER_INCH);
//            leftFront.setTargetPosition(newLeftFrontTarget);
//            leftRear.setTargetPosition(newLeftRearTarget);
//            rightFront.setTargetPosition(newRightFrontTarget);
//            rightRear.setTargetPosition(newRightRearTarget);
//
//            // Turn On RUN_TO_POSITION
//            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            leftFront.setPower(Math.abs(speed));
//            leftRear.setPower(Math.abs(speed));
//            rightFront.setPower(Math.abs(speed));
//            rightRear.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftRearTarget, newRightFrontTarget, newRightRearTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
//                        leftFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightFront.getCurrentPosition(), rightRear.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            leftFront.setPower(0);
//            leftRear.setPower(0);
//            rightFront.setPower(0);
//            rightRear.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move.
//        }
//    } //^Under construction to borrow code to move the lifts with encoder counts^
    }
}h
