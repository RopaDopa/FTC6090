package org.firstinspires.ftc.FinalBotCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class MecanumRobot extends LinearOpMode {
    MHardware robot = new MHardware(this);

    public void runOpMode() {

        robot.init();
        waitForStart();

        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            robot.driveRobot(axial, lateral, yaw);

            //~~~~Drone mechanisms
            if(gamepad2.a) { //angles drone up
                robot.angleDrone(MHardware.ANGLE_DRONE);
            }

            if(gamepad2.left_bumper) { //releases drone
                robot.launchDrone(MHardware.LAUNCH_DRONE);
            }
            else { //holding launcher
               robot.launchDrone(0);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            //~~~~Intake Mechanisms
            if(gamepad2.right_bumper) {
                robot.grabPixel(MHardware.GRAB_PIXEL);
            }
            else {
                robot.releasePixel(MHardware.RELEASE_PIXEL);
            }

           if(Math.abs(gamepad2.left_trigger) > 0) {
               robot.liftIntake(MHardware.LIFT_INTAKE);
           }
           else if(Math.abs(gamepad2.right_trigger) > 0) {
               robot.liftIntake(-MHardware.LIFT_INTAKE);
           }
           else {
               robot.liftIntake(0);
           }

            if(gamepad2.b) { //dw
                robot.intakeLiftAngle(MHardware.INTAKELIFT_BACKPOS);
            }
            if(gamepad2.y) { //dw
                robot.intakeLiftAngle(MHardware.INTAKELIFT_STRAIGHTPOS);
            }
            if(gamepad2.x) { //dw
                robot.intakeLiftAngle(MHardware.INTAKELIFT_FORWARDPOS);
            }

            if(gamepad2.right_stick_button) {
                robot.intakeAngle(MHardware.INTAKE_COLLECTPOS);
            }
            if(gamepad2.left_stick_button) { //dw
                robot.intakeAngle(MHardware.INTAKE_BBPOS);
            }
            if(gamepad2.dpad_right) {
                robot.intakeAngle(MHardware.INTAKE_RESTPOS);
            }
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


            //~~~~~~Hanging mechanism
            if(gamepad2.dpad_up) {
                robot.hangRobot(0.5, 15, 3);
            }

            if(gamepad2.dpad_down) {
                robot.hangRobot(0.5, -15, 3);
            }
            //~~~~~~~~~~~~~~~~~~~

            sleep(50);

        }
        //dw = doesn't work

    }
}
