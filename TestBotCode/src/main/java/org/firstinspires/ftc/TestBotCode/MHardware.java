package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MHardware {
    //This class is for putting the hardware separate from the other stuff. Everything here would go in the runOpMode()
    private LinearOpMode jrOpMode = null;
    private DcMotor leftFrontDrive = null; //Tetrix TorqueNADO
    private DcMotor leftBackDrive = null; //Tetrix TorqueNADO
    private DcMotor rightFrontDrive = null; //Tetrix TorqueNADO
    private DcMotor rightBackDrive = null; //Tetrix TorqueNADO

    public MHardware(LinearOpMode opmode) {
        jrOpMode = opmode;
    }

    public void init() {
        leftFrontDrive = jrOpMode.hardwareMap.get(DcMotor.class, "leftf_drive");
        leftBackDrive = jrOpMode.hardwareMap.get(DcMotor.class, "leftb_drive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive = jrOpMode.hardwareMap.get(DcMotor.class, "rightf_drive");
        rightBackDrive = jrOpMode.hardwareMap.get(DcMotor.class, "rightb_drive");
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        jrOpMode.telemetry.addData("Status:", "Robot Ready");
        jrOpMode.telemetry.update();


    }
    public void setPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(leftBackPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(rightBackPower));

        if(maxSpeed > 1.0) {
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

    public void driveRobot(double axial, double lateral, double yaw) {
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        setPowers(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
    }


}
