package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="KludgeTeleOpGood", group="Linear OpMode")
public class KludgeTeleOpGood extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Reverse necessary motors
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Driving controls
            double axial = gamepad1.left_stick_y;  // Forward/backward
            double lateral = gamepad1.left_stick_x; // Left/right strafing
            double yaw = gamepad1.right_stick_x;    // Rotation

            /*
            double SpeedVariable=1;
            if(gamepad1.dpad_up);
            SpeedVariable=1;

            if(gamepad1.dpad_down);
            SpeedVariable=0.5;

            if(gamepad1.dpad_left);
            SpeedVariable=0.25;

            if(gamepad1.dpad_up);
            SpeedVariable=0.75;
            */

            // Compute motor powers
            double FrontLeftPower = axial + lateral + yaw;
            double FrontRightPower = axial - lateral - yaw;
            double BackLeftPower = axial - lateral + yaw;
            double BackRightPower = axial + lateral - yaw;

            // Normalize the powers so no value exceeds 1.0
            double maxPower = Math.max(Math.abs(FrontLeftPower), Math.abs(FrontRightPower));
            maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
            maxPower = Math.max(maxPower, Math.abs(BackRightPower));
            if (maxPower > 1) {
                FrontLeftPower /= maxPower;
                FrontRightPower /= maxPower;
                BackLeftPower /= maxPower;
                BackRightPower /= maxPower;
            }

            // Set motor powers
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);

            // Telemetry for debugging
            telemetry.addData("FrontLeft Power", FrontLeftPower);
            telemetry.addData("FrontRight Power", FrontRightPower);
            telemetry.addData("BackLeft Power", BackLeftPower);
            telemetry.addData("BackRight Power", BackRightPower);
            telemetry.update();
        }
    }
}