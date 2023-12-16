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

    private ElapsedTime     runtime = new ElapsedTime();

    //Constants
    public static final double GRAB_PIXEL = 0;
    public static final double RELEASE_PIXEL = 45;
    public static final double ANGLE_DRONE = 45;
    public static final double ANGLE_DRONE_RESTPOS = 45;
    public static final double LAUNCH_DRONE = 90;
    public static final double INTAKELIFT_BACKPOS = 0;
    public static final double INTAKELIFT_STRAIGHTPOS = 45;
    public static final double INTAKELIFT_FORWARDPOS = 90;
    public static final double INTAKE_COLLECTPOS = 90;
    public static final double INTAKE_BBPOS = 45;
    public static final double INTAKE_RESTPOS = 0;
    public static final double LIFT_INTAKE = 0.8;
    public static final double HANG_ROBOT = 0.5;
    public static final double HANGLIFT_COUNTSPERINCH = 2357;

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
    public void hangRobot(double speed, double hangLiftInches, double timeoutS) {
        int newHangMotorTarget;
        double hangLiftInchesTotal;

        newHangMotorTarget = hangMotor.getCurrentPosition() + (int)(hangLiftInches * HANGLIFT_COUNTSPERINCH);

        hangMotor.setTargetPosition(newHangMotorTarget);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        hangMotor.setPower(Math.abs(speed));

        while (finalOpMode.opModeIsActive() && (runtime.seconds() < timeoutS) && (hangMotor.isBusy())) {

            // Display it for the driver.
            finalOpMode.telemetry.addData("Running to", "%7d", newHangMotorTarget);
            finalOpMode.telemetry.addData("Currently at", "at %7d", hangMotor.getCurrentPosition());
            finalOpMode.telemetry.update();

        }
        // Stop all motion;
        hangMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        finalOpMode.sleep(250);
    }

    //LIFT THE INTAKE  (motor)
    public void liftIntake(double lift) {
        liftIntakeMotor.setPower(lift);
    } //^UNDER CONSTRUCTION TO MOVE LIFT USING ENCODER COUNTS^

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
    public void intakeLiftAngle(double liftPosition) {
        angleIntakeLiftServo.setPosition(liftPosition);
    }

    //ANGLE THE INTAKE (linear actuator)
    public void intakeAngle(double position) {
        angleIntakeServo.setPosition(position);
    }
}
