package org.firstinspires.ftc.FinalBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class EncoderTest extends LinearOpMode {
    //Variables to store the hardware position for the motors
    private DcMotor liftIntakeMotor = null;
    private DcMotor hangMotor = null;
    public void runOpMode() {
        //Getting the motors from the hardware configuration
        liftIntakeMotor = hardwareMap.get(DcMotor.class, "liftIntakeMotor");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");

        liftIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        hangMotor.setDirection(DcMotor.Direction.REVERSE);

        //Resetting the previous encoder counts
        liftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting the motor to run with encoders
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Variables to store the position before starting
        double startPos = liftIntakeMotor.getCurrentPosition();
        double hangStartPos = hangMotor.getCurrentPosition();

        //Telemetry to check if the encoder counts reset to 0
        telemetry.addData("Intake lift is starting at", startPos);
        telemetry.addData("Hang lift is starting at", hangStartPos);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //Storing the controls to move the motors in variables.
            double drive = -gamepad1.left_stick_y;
            double hang = -gamepad1.right_stick_y;

            //Storing value of mode of motors in string to use in telemetry
            String hangMotorMode = String.valueOf(hangMotor.getMode());
            String liftIntakeMotorMode = String.valueOf(liftIntakeMotor.getMode());

            //Allowing to motor to move based on the gamepad input
            liftIntakeMotor.setPower(Math.abs(drive));
            hangMotor.setPower(Math.abs(hang));

            //Telemetry to see the encoder counts
            telemetry.addData("Lift Running Mode", liftIntakeMotorMode);
            telemetry.addData("Hang Motor Running Mode", hangMotorMode);
            telemetry.addData("~~~~~~~~~", "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
            telemetry.addData("Lift Encoder Counts", "%7d", liftIntakeMotor.getCurrentPosition());
            telemetry.addData("Hang Motor Encoder Counts", "%7d", hangMotor.getCurrentPosition());
            telemetry.update();

            sleep(50);
        }
    }
}
