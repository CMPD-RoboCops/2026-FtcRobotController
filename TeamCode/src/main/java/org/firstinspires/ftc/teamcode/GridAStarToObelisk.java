package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;

/**
 * ======================================================
 * Autonomous Path Planning using A* (No IMU)
 * ------------------------------------------------------
 * Drives a mecanum robot from a start grid cell to a goal
 * using A* pathfinding + encoder-based localization.
 * ======================================================
 */
@Autonomous(name = "Grid A* to Obelisk (Fixed Clean Version)(descriptive)(no EMU)", group = "Competition")
public class GridAStarToObelisk extends LinearOpMode {

    // ---------------- GRID SETTINGS ----------------
    static final int GRID_SIZE = 48;
    static final double CELL_INCHES = 3.0;

    // ---------------- ROBOT CONSTANTS ----------------
    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_RADIUS_IN = 1.889;
    static final double GEAR_RATIO = 1.0;
    static final double TRACK_WIDTH_IN = 14.0;
    static final double LATERAL_MULTIPLIER = 1.08;

    // Control tuning
    static final double VX_kP = 0.085;
    static final double VY_kP = 0.085;
    static final double HEADING_kP = 0.015;

    static final double MAX_DRIVE_POWER = 0.7;
    static final double MAX_TURN_POWER = 0.5;

    static final double POSITION_TOLERANCE_IN = 2.0;
    static final double HEADING_TOLERANCE_DEG = 7.0;

    // Start and goal cells
    static final int START_CX = 46, START_CY = 2;
    static final int GOAL_CX = 24, GOAL_CY = 46;

    // ---------------- HARDWARE ----------------
    private DcMotor fl, fr, bl, br;

    // Localization
    private int lastFL, lastFR, lastBL, lastBR;
    private double lastHeadingRad = 0.0;
    private Pose2d pose = new Pose2d(
            (START_CX + 0.5) * CELL_INCHES,
            (START_CY + 0.5) * CELL_INCHES,
            0.0
    );

    private int[][] grid = new int[GRID_SIZE][GRID_SIZE];

    @Override
    public void runOpMode() {
        // ---------------- MAP SETUP ----------------
        initDefaultGrid(grid);
        tryLoadCsvGrid("/sdcard/FIRST/ftc_field_grid_3in.csv", grid);

        if (isBlocked(grid, START_CX, START_CY) || isBlocked(grid, GOAL_CX, GOAL_CY)) {
            telemetry.addLine("Start or Goal is blocked. Fix map or cells.");
            telemetry.update();
            return;
        }

        // ---------------- HARDWARE INIT ----------------
        fl = hardwareMap.get(DcMotor.class, "FrontLeft");
        fr = hardwareMap.get(DcMotor.class, "FrontRight");
        bl = hardwareMap.get(DcMotor.class, "BackLeft");
        br = hardwareMap.get(DcMotor.class, "BackRight");

        // For standard mecanum orientation:
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();

        telemetry.addLine("A* Planner Ready (No IMU)");
        telemetry.addData("Start", "(%d, %d)", START_CX, START_CY);
        telemetry.addData("Goal", "(%d, %d)", GOAL_CX, GOAL_CY);
        telemetry.update();

        // ---------------- PATH PLANNING PREVIEW ----------------
        List<Cell> path = aStar(grid, new Cell(START_CX, START_CY), new Cell(GOAL_CX, GOAL_CY));
        if (path == null || path.isEmpty()) {
            telemetry.addLine("No path found!");
            telemetry.update();
            return;
        }

        List<Cell> prunedPath = prunePathWithLoS(grid, path);
        List<Pose2d> waypoints = toWaypointList(prunedPath);

        telemetry.addData("Path Cells", path.size());
        telemetry.addData("After Prune", prunedPath.size());
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- FOLLOW WAYPOINTS ----------------
        for (Pose2d target : waypoints) {
            goToWaypoint(target);
            if (!opModeIsActive()) break;
        }

        driveMecanum(0, 0, 0);
        sleep(200);
    }

    // ------------------------- PATH PLANNING -------------------------

    List<Cell> aStar(int[][] g, Cell start, Cell goal) {
        int w = g[0].length, h = g.length;
        boolean[][] closed = new boolean[h][w];
        Cell[][] parent = new Cell[h][w];
        double[][] gScore = new double[h][w];
        for (double[] row : gScore) Arrays.fill(row, Double.POSITIVE_INFINITY);
        PriorityQueue<Cell> open = new PriorityQueue<>(Comparator.comparingDouble(c -> c.f));

        start.g = 0;
        start.f = hCost(start, goal);
        gScore[start.y][start.x] = 0;
        open.add(start);

        int[][] dirs = {
                {1, 0}, {-1, 0}, {0, 1}, {0, -1},
                {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        while (!open.isEmpty()) {
            Cell cur = open.poll();
            if (closed[cur.y][cur.x]) continue;
            if (cur.x == goal.x && cur.y == goal.y) {
                return reconstruct(parent, cur);
            }

            closed[cur.y][cur.x] = true;

            for (int[] d : dirs) {
                int nx = cur.x + d[0];
                int ny = cur.y + d[1];
                if (!inBounds(nx, ny, w, h)) continue;
                if (isBlocked(g, nx, ny)) continue;
                if (d[0] != 0 && d[1] != 0) {
                    if (isBlocked(g, cur.x + d[0], cur.y) || isBlocked(g, cur.x, cur.y + d[1]))
                        continue;
                }

                double step = (d[0] == 0 || d[1] == 0) ? 1.0 : Math.sqrt(2);
                double tentativeG = gScore[cur.y][cur.x] + step;

                if (tentativeG < gScore[ny][nx]) {
                    gScore[ny][nx] = tentativeG;
                    double f = tentativeG + hCost(nx, ny, goal.x, goal.y);
                    Cell n = new Cell(nx, ny, tentativeG, f);
                    parent[ny][nx] = cur;
                    open.add(n);
                }
            }
        }
        return null;
    }

    List<Cell> reconstruct(Cell[][] parent, Cell end) {
        ArrayList<Cell> out = new ArrayList<>();
        Cell cur = end;
        out.add(cur);
        while (true) {
            Cell p = parent[cur.y][cur.x];
            if (p == null) break;
            out.add(p);
            cur = p;
        }
        Collections.reverse(out);
        return out;
    }

    List<Cell> prunePathWithLoS(int[][] g, List<Cell> path) {
        if (path.size() <= 2) return path;
        ArrayList<Cell> pruned = new ArrayList<>();
        int i = 0;
        pruned.add(path.get(0));
        int j = 1;
        while (j < path.size()) {
            if (hasLineOfSight(g, path.get(i), path.get(j))) {
                j++;
            } else {
                pruned.add(path.get(j - 1));
                i = j - 1;
            }
        }
        pruned.add(path.get(path.size() - 1));

        ArrayList<Cell> cleaned = new ArrayList<>();
        Cell last = null;
        for (Cell c : pruned) {
            if (last == null || !(last.x == c.x && last.y == c.y)) cleaned.add(c);
            last = c;
        }
        return cleaned;
    }

    boolean hasLineOfSight(int[][] g, Cell a, Cell b) {
        for (int[] p : bresenham(a.x, a.y, b.x, b.y)) {
            if (isBlocked(g, p[0], p[1])) return false;
        }
        return true;
    }

    List<int[]> bresenham(int x0, int y0, int x1, int y1) {
        ArrayList<int[]> pts = new ArrayList<>();
        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;
        while (true) {
            pts.add(new int[]{x, y});
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
        return pts;
    }

    List<Pose2d> toWaypointList(List<Cell> cells) {
        ArrayList<Pose2d> list = new ArrayList<>();
        for (Cell c : cells) {
            list.add(new Pose2d((c.x + 0.5) * CELL_INCHES, (c.y + 0.5) * CELL_INCHES, 0.0));
        }
        for (int i = 0; i < list.size() - 1; i++) {
            Pose2d a = list.get(i), b = list.get(i + 1);
            double hdg = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
            a.h = hdg;
        }
        if (!list.isEmpty()) list.get(list.size() - 1).h = 0.0;
        return list;
    }

    // ------------------------- DRIVING -------------------------

    private void goToWaypoint(Pose2d targetPose) {
        while (opModeIsActive()) {
            updateLocalPoseFromEncoders();

            double dx = targetPose.x - pose.x;
            double dy = targetPose.y - pose.y;
            double distanceError = Math.hypot(dx, dy);

            double headingRad = Math.toRadians(pose.h);
            double robotXError = Math.cos(headingRad) * dx + Math.sin(headingRad) * dy;
            double robotYError = -Math.sin(headingRad) * dx + Math.cos(headingRad) * dy;

            double headingError = normalizeDeg(targetPose.h - pose.h);

            double forwardCmd = VX_kP * robotXError;
            double strafeCmd = VY_kP * robotYError * LATERAL_MULTIPLIER;
            double turnCmd = HEADING_kP * headingError;
            turnCmd = clamp(turnCmd, -MAX_TURN_POWER, MAX_TURN_POWER);

            if (distanceError < POSITION_TOLERANCE_IN && Math.abs(headingError) < HEADING_TOLERANCE_DEG) {
                driveMecanum(0, 0, 0);
                break;
            }

            driveMecanum(forwardCmd, strafeCmd, turnCmd);

            telemetry.addData("Target", "(%.1f, %.1f, %.1f°)", targetPose.x, targetPose.y, targetPose.h);
            telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", pose.x, pose.y, pose.h);
            telemetry.update();
        }
    }

    private void driveMecanum(double forward, double strafe, double turn) {
        double flPower = forward + strafe + turn;
        double frPower = forward - strafe - turn;
        double blPower = forward - strafe + turn;
        double brPower = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(flPower),
                Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower((flPower / max) * MAX_DRIVE_POWER);
        fr.setPower((frPower / max) * MAX_DRIVE_POWER);
        bl.setPower((blPower / max) * MAX_DRIVE_POWER);
        br.setPower((brPower / max) * MAX_DRIVE_POWER);
    }

    // ------------------------- LOCALIZATION -------------------------

    private void updateLocalPoseFromEncoders() {
        int curFL = fl.getCurrentPosition();
        int curFR = fr.getCurrentPosition();
        int curBL = bl.getCurrentPosition();
        int curBR = br.getCurrentPosition();

        int dFL = curFL - lastFL;
        int dFR = curFR - lastFR;
        int dBL = curBL - lastBL;
        int dBR = curBR - lastBR;

        lastFL = curFL;
        lastFR = curFR;
        lastBL = curBL;
        lastBR = curBR;

        double ticksToIn = (2.0 * Math.PI * WHEEL_RADIUS_IN) * GEAR_RATIO / TICKS_PER_REV;
        double dFLin = dFL * ticksToIn;
        double dFRin = dFR * ticksToIn;
        double dBLin = dBL * ticksToIn;
        double dBRin = dBR * ticksToIn;

        double dXr = (dFLin + dFRin + dBLin + dBRin) / 4.0;
        double dYr = (-dFLin + dFRin + dBLin - dBRin) / 4.0 * LATERAL_MULTIPLIER;

        double leftAvg = (dFLin + dBLin) / 2.0;
        double rightAvg = (dFRin + dBRin) / 2.0;
        double dTheta = (rightAvg - leftAvg) / TRACK_WIDTH_IN;
        lastHeadingRad += dTheta;

        double cos = Math.cos(lastHeadingRad);
        double sin = Math.sin(lastHeadingRad);
        double dXf = cos * dXr - sin * dYr;
        double dYf = sin * dXr + cos * dYr;

        pose = new Pose2d(pose.x + dXf, pose.y + dYf, Math.toDegrees(lastHeadingRad));
    }

    // ------------------------- UTILITIES -------------------------

    void initDefaultGrid(int[][] g) {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                g[y][x] = 0;
            }
        }
        for (int i = 0; i < GRID_SIZE; i++) {
            g[0][i] = 1;
            g[GRID_SIZE - 1][i] = 1;
            g[i][0] = 1;
            g[i][GRID_SIZE - 1] = 1;
        }
    }

    void tryLoadCsvGrid(String path, int[][] g) {
        try {
            File f = new File(path);
            if (!f.exists()) return;
            BufferedReader br = new BufferedReader(new FileReader(f));
            ArrayList<String[]> rows = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null) rows.add(line.split(","));
            br.close();
            if (rows.size() != GRID_SIZE || rows.get(0).length != GRID_SIZE) return;

            for (int y = 0; y < GRID_SIZE; y++) {
                for (int x = 0; x < GRID_SIZE; x++) {
                    g[y][x] = (rows.get(y)[x].trim().equals(".")) ? 0 : 1;
                }
            }
        } catch (Exception ignored) {
        }
    }

    boolean inBounds(int x, int y, int w, int h) {
        return x >= 0 && y >= 0 && x < w && y < h;
    }

    boolean isBlocked(int[][] g, int x, int y) {
        return g[y][x] >= 1;
    }

    double hCost(Cell a, Cell b) {
        return hCost(a.x, a.y, b.x, b.y);
    }

    double hCost(int x, int y, int gx, int gy) {
        int dx = Math.abs(x - gx), dy = Math.abs(y - gy);
        int diag = Math.min(dx, dy);
        int straight = Math.max(dx, dy) - diag;
        return diag * Math.sqrt(2) + straight;
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    static double normalizeDeg(double a) {
        while (a <= -180) a += 360;
        while (a > 180) a -= 360;
        return a;
    }

    // ------------------------- DATA STRUCTURES -------------------------
    static class Cell {
        int x, y;
        double g, f;

        Cell(int x, int y) {
            this.x = x;
            this.y = y;
        }

        Cell(int x, int y, double g, double f) {
            this.x = x;
            this.y = y;
            this.g = g;
            this.f = f;
        }
    }

    static class Pose2d {
        double x, y, h;

        Pose2d(double x, double y, double h) {
            this.x = x;
        }
    }
}