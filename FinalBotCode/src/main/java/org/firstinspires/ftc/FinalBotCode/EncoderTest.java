package org.firstinspires.ftc.FinalBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncoderTest extends LinearOpMode {
    //Variables to store the hardware position for the motors
    private DcMotor liftIntakeMotor = null;
    private DcMotor hangMotor = null;
    public void runOpMode() {
        //Getting the motors fromt the hardware configuration
        liftIntakeMotor = hardwareMap.get(DcMotor.class, "liftIntakeMotor");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");

        //Resetting the previous encoder counts
        liftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Setting the motors to run with encoders
        liftIntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            double drive = gamepad1.left_stick_y;
            double hang = gamepad1.right_stick_y;

            //Assigning the position of the motors (the current encoder counts) to variables
            double position = liftIntakeMotor.getCurrentPosition();
            double hangPos = hangMotor.getCurrentPosition();

            //Storing value of mode of motors in string to use in telemetry (don't know if it works)
            String hangMotorMode = String.valueOf(hangMotor.getMode());
            String liftIntakeMotorMode = String.valueOf(liftIntakeMotor.getMode());

            //Allowing to motor to move based on the gamepad input
            liftIntakeMotor.setPower(drive);
            hangMotor.setPower(hang);

            //Telemetry to see the encoder counts
            telemetry.addData("Lift Running Mode", liftIntakeMotorMode);
            telemetry.addData("Hang Motor Running Mode", hangMotorMode);
            telemetry.addData("Lift Encoder Counts", position);
            telemetry.addData("Hang Motor Encoder Counts", hangPos);
            telemetry.update();

            sleep(50);
        }
    }
}
