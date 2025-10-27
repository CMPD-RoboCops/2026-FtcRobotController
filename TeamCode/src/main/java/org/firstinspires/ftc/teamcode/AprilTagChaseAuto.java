package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

//This Op Mode is written for:  Kludge

@Autonomous(name="AprilTag Chase (Demo)")
public class AprilTagChaseAuto extends LinearOpMode {

    // === MOTORS (rename to your config) ===
    DcMotor lf, rf, lb, rb;

    // === CAMERA / TAG PROCESSOR ===
    VisionPortal portal;
    AprilTagProcessor tagProc;

    // === CAMERA INTRINSICS (TODO: replace with your calibration) ===
    static final double FX = 820, FY = 820, CX = 320, CY = 240;   // example values
    // If you have distortion coeffs, add .setDistortionCoefficients() below.

    // === CAMERA MOUNT ON ROBOT (meters & degrees; TODO: measure yours) ===
    static final double CAM_X = 0.15, CAM_Y = 0.00, CAM_Z = 0.20;
    static final double CAM_ROLL = 0.0, CAM_PITCH = 0.0, CAM_YAW = 0.0;

    // === IMAGE GEOMETRY (TODO: set to your actual stream resolution/FOV) ===
    static final int IMG_W = 640, IMG_H = 480;
    // If your SDK doesn’t expose a bearing, we’ll estimate it from pixels using HFOV:
    static final double CAMERA_HFOV_DEG = 70.0; // replace with your real value

    // === CONTROL GAINS / LIMITS (tune these) ===
    static final double K_TURN = 0.75;           // motor power per degree of bearing error, Original Value 0.02
    static final double K_FWD  = 0.75;           // base forward power, Original Value 0.40
    static final double FWD_MIN = 0.75;          // minimum creep forward when allowed, Original Value 0.10
    static final double FWD_MAX = 1;          // cap forward speed, Original Value 0.40

    static final double BEARING_DEADBAND_DEG = 2.0;   // don’t turn if within this
    static final double RANGE_SLOWDOWN_START_M = 1.0; // start slowing forward power when closer than this
    static final double STOP_RANGE_M = 0.35;          // stop when this close to the tag plane (tune!)

    // If no range available, we approximate “closeness” by tag pixel size:
    static final double TAG_SIZE_PIX_TO_STOP = 99999;   // area (px^2) at which we stop, Original is 14000
    static final double TAG_SIZE_PIX_TO_SLOW = 1;       // start slowing forward

    // === SCAN BEHAVIOR WHEN NO TAG SEEN ===
    static final double SCAN_TURN_POWER = 0.0;   // gentle auto-scan if > 0.0
    static final double SCAN_DIR_SECONDS = 3;

    // === SIGNS/CALIBRATION ===
    // If +fwd makes the robot back up, set FWD_SIGN = -1.
    static final int FWD_SIGN  = -1;
    // If seeing the tag on the right makes the robot turn left, set TURN_SIGN = -1.
    static final int TURN_SIGN = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        // ----- Hardware -----
        lf = hardwareMap.get(DcMotor.class, "FrontLeft");
        rf = hardwareMap.get(DcMotor.class, "FrontRight");
        lb = hardwareMap.get(DcMotor.class, "BackLeft");
        rb = hardwareMap.get(DcMotor.class, "BackRight");

        // Mecanum standard directions; flip if your robot drives backward
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{lf, rf, lb, rb}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // ----- Vision -----
        AprilTagLibrary fieldTags = AprilTagGameDatabase.getCurrentGameTagLibrary(); // season library

        tagProc = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setTagLibrary(fieldTags)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(FX, FY, CX, CY)
                // .setDistortionCoefficients(k1,k2,p1,p2,k3) // if you have them
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(tagProc)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        // This version of the SDK may not include this
        // tagProc.setCameraPose(CAM_X, CAM_Y, CAM_Z, CAM_ROLL, CAM_PITCH, CAM_YAW,
        //        DistanceUnit.METER, AngleUnit.DEGREES);

        telemetry.addLine("AprilTag Chase: READY");
        telemetry.update();

        waitForStart();

        double lastScanFlipTime = getRuntime();
        int scanDir = 1;

        while (opModeIsActive()) {
            // Defensive copy to avoid ConcurrentModificationException
            List<AprilTagDetection> dets = new ArrayList<>(tagProc.getDetections());

            if (!dets.isEmpty()) {
                // Pick the "best" detection: highest decision margin, else largest area
                dets.sort(Comparator.<AprilTagDetection>comparingDouble(d -> -d.decisionMargin));
                AprilTagDetection best = dets.get(0);

                // === 1) Compute BEARING error (deg) ===
                Double bearingDeg = null;

                // Path A: use SDK bearing if available
                try {
                    bearingDeg = best.ftcPose.bearing;
                } catch (Throwable t) {
                    // ignore, fall back to pixel-based
                }

                // Path B: estimate from pixel center using known HFOV
                if (bearingDeg == null) {
                    double cxPix = (best.center != null) ? best.center.x : avgX(best);
                    double norm = (cxPix - (IMG_W / 2.0)) / (IMG_W / 2.0); // -1..+1
                    bearingDeg = norm * (CAMERA_HFOV_DEG / 2.0);
                }

                // === 2) Estimate RANGE / CLOSENESS for forward speed scaling ===
                Double rangeM = null;
                try {
                    rangeM = best.ftcPose.range; // if available (meters)
                } catch (Throwable t) {
                    // ignore
                }

                double closeness = 0.0; // 0 far … 1 very close
                if (rangeM != null) {
                    if (rangeM <= STOP_RANGE_M) closeness = 1.0;
                    else if (rangeM < RANGE_SLOWDOWN_START_M) {
                        double span = RANGE_SLOWDOWN_START_M - STOP_RANGE_M;
                        closeness = 1.0 - Math.max(0, (rangeM - STOP_RANGE_M) / span);
                    } else {
                        closeness = 0.0;
                    }
                } else {
                    // fall back to tag area in pixels
                    double area = tagAreaPx(best);
                    if (area >= TAG_SIZE_PIX_TO_STOP) closeness = 1.0;
                    else if (area > TAG_SIZE_PIX_TO_SLOW) {
                        double span = TAG_SIZE_PIX_TO_STOP - TAG_SIZE_PIX_TO_SLOW;
                        closeness = Math.max(0.0, (area - TAG_SIZE_PIX_TO_SLOW) / span);
                    } else {
                        closeness = 0.0;
                    }
                }

                // === 3) Controllers ===
                double turnCmd = 0.0;
                double absErr = Math.abs(bearingDeg);

                if (absErr > BEARING_DEADBAND_DEG) {
                    // Apply TURN_SIGN so tag on right results in right turn
                    turnCmd = clamp(K_TURN * TURN_SIGN * bearingDeg, -0.4, 0.4);
                }

                // Forward power: creep forward when roughly centered, slow as we get close
                double fwdBase = (absErr <= BEARING_DEADBAND_DEG) ? K_FWD : 0.0; // only drive when centered-ish
                double slowFactor = 1.0 - clamp(closeness, 0.0, 1.0);            // 1 far → 0 at stop
                double fwdCmd = clamp(fwdBase * slowFactor, 0.0, FWD_MAX);

                // keep a minimum creep when allowed
                if (fwdCmd > 0 && fwdCmd < FWD_MIN) fwdCmd = FWD_MIN;

                // Hard stop if within stop range
                boolean shouldStop = (rangeM != null && rangeM <= STOP_RANGE_M)
                        || (rangeM == null && tagAreaPx(best) >= TAG_SIZE_PIX_TO_STOP);
                if (shouldStop) {
                    fwdCmd = 0.0;
                    turnCmd = 0.0;
                }

                driveArcade(fwdCmd, turnCmd);

                telemetry.addData("Tag ID", best.id);
                telemetry.addData("Bearing (deg)", "%.1f", bearingDeg);
                telemetry.addData("Range (m)", (rangeM==null) ? "n/a" : String.format("%.2f", rangeM));
                telemetry.addData("Turn", "%.2f", turnCmd);
                telemetry.addData("Fwd", "%.2f", fwdCmd);
                telemetry.addData("Close(0..1)", "%.2f", closeness);
                telemetry.update();

            } else {
                // No tag: slow scan to acquire (set SCAN_TURN_POWER > 0 to enable)
                double now = getRuntime();
                if (now - lastScanFlipTime > SCAN_DIR_SECONDS) {
                    scanDir *= -1;
                    lastScanFlipTime = now;
                }
                driveArcade(0.0, SCAN_TURN_POWER * scanDir);

                telemetry.addLine("No tag detected. Scanning…");
                telemetry.update();
            }
        }

        stopDrive();
    }

    // --- helpers ---

    void driveArcade(double fwd, double turn) {
        // Ensure +fwd = move toward the tag (flip once here if needed)
        fwd *= FWD_SIGN;

        // No strafe; simple mecanum/tank mix
        double lfP = fwd + turn;
        double rfP = fwd - turn;
        double lbP = fwd + turn;
        double rbP = fwd - turn;

        // normalize to [-1,1]
        double max = Math.max(1.0, Math.max(Math.abs(lfP),
                Math.max(Math.abs(rfP), Math.max(Math.abs(lbP), Math.abs(rbP)))));
        lf.setPower(lfP / max);
        rf.setPower(rfP / max);
        lb.setPower(lbP / max);
        rb.setPower(rbP / max);
    }

    void stopDrive() {
        lf.setPower(0); rf.setPower(0); lb.setPower(0); rb.setPower(0);
    }

    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    static double avgX(AprilTagDetection d) {
        // If your SDK exposes corners[] with (x,y)
        try {
            return (d.corners[0].x + d.corners[1].x + d.corners[2].x + d.corners[3].x) / 4.0;
        } catch (Throwable t) {
            // If only "center" exists but we failed above, return image center as fallback
            return IMG_W / 2.0;
        }
    }

    static double tagAreaPx(AprilTagDetection d) {
        try {
            // polygon area from corners (shoelace)
            double x0 = d.corners[0].x, y0 = d.corners[0].y;
            double x1 = d.corners[1].x, y1 = d.corners[1].y;
            double x2 = d.corners[2].x, y2 = d.corners[2].y;
            double x3 = d.corners[3].x, y3 = d.corners[3].y;
            double sum = (x0*y1 - y0*x1) + (x1*y2 - y1*x2) + (x2*y3 - y2*x3) + (x3*y0 - y3*x0);
            return Math.abs(sum) * 0.5;
        } catch (Throwable t) {
            // fallback: approximate 0 if unavailable
            return 0.0;
        }
    }
}
