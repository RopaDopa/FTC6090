package org.firstinspires.ftc.FinalBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncoderTest extends LinearOpMode {
    private DcMotor testMotor = null;

    @Override
    public void runOpMode() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testMotor.setDirection(DcMotor.Direction.REVERSE);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at", "%7d :%7d", testMotor.getCurrentPosition());

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            testMotor.setPower(drive);

            telemetry.addData("Encoder Counts", testMotor.getCurrentPosition());

            sleep(50);
        }
    }

}
