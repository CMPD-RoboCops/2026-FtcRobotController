package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.*;

/**
 * A* grid navigation (48x48, 3" cells) with waypoint pruning + mecanum driving.
 *
 * Map legend (int):
 *   0 = free, 1 = wall/blocked, 2+ = higher traversal cost (optional; we treat >=1 as blocked by default)
 *
 * Default map: empty interior + perimeter walls. You can override by placing a CSV at:
 *   /sdcard/FIRST/ftc_field_grid_3in.csv
 * (48 rows x 48 columns of single characters; any non-dot treated as blocked.)
 *
 * Start/goal (cells) can be edited below. Current route:
 *   Start near Blue alliance front-right corner  ->  Obelisk near back-center (one cell off wall).
 */
@Autonomous(name = "Grid A* to Obelisk (48x48 @3\")", group = "Competition")
public class GridAStarToObelisk extends LinearOpMode {

    // ---------------- GRID & SCALE ----------------
    static final int GRID = 48;          // 48x48 cells
    static final double CELL_IN = 3.0;   // 3 inches per cell

    // ---------------- ROBOT CONSTANTS (TUNE) ----------------
    public static final double TICKS_PER_REV      = 537.7;
    public static final double WHEEL_RADIUS_IN    = 1.889; // ~3.78" dia wheels
    public static final double GEAR_RATIO         = 1.0;
    public static final double TRACK_WIDTH_IN     = 14.0;
    public static final double LATERAL_MULTIPLIER = 1.08;

    public static final double VX_kP = 0.085;
    public static final double VY_kP = 0.085;
    public static final double HEADING_kP = 0.015;
    public static final double MAX_DRIVE_POWER = 0.7;
    public static final double MAX_TURN_POWER  = 0.5;
    public static final double POS_TOL_IN      = 2.0;
    public static final double HEADING_TOL_DEG = 7.0;

    // ---------------- START / GOAL (CELLS) ----------------
    // Start a tad off the walls; goal a tad off back wall at center x.
    static final int START_CX = 46, START_CY = 2;   // Blue front-right inside corner-ish
    static final int GOAL_CX  = 24, GOAL_CY  = 46;  // ~ back-center, 6" off back wall

    // ---------------- HW ----------------
    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;

    // Odometry
    private int lastFL, lastFR, lastBL, lastBR;
    private Pose2d pose = new Pose2d((START_CX + 0.5) * CELL_IN, (START_CY + 0.5) * CELL_IN, 0.0);

    // Map
    private int[][] grid = new int[GRID][GRID];

    @Override
    public void runOpMode() {
        // ---- Load grid ----
        initDefaultGrid(grid);                      // perimeter walls
        tryLoadCsvGrid("/sdcard/FIRST/ftc_field_grid_3in.csv", grid); // optional override

        // ---- Hardware map ----
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(p);

        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();

        telemetry.addLine("A* ready");
        telemetry.addData("Start cell", "(%d,%d)", START_CX, START_CY);
        telemetry.addData("Goal cell",  "(%d,%d)", GOAL_CX,  GOAL_CY);
        telemetry.update();

        // ---- PLAN PATH ----
        List<Cell> pathCells = aStar(grid, new Cell(START_CX, START_CY), new Cell(GOAL_CX, GOAL_CY));
        if (pathCells == null || pathCells.isEmpty()) {
            telemetry.addLine("No path found!");
            telemetry.update();
        } else {
            // prune to fewer waypoints using line-of-sight (grid Bresenham)
            List<Cell> prunedCells = prunePathWithLoS(grid, pathCells);
            List<Pose2d> waypoints = toWaypointList(prunedCells);

            telemetry.addData("Cells in path", pathCells.size());
            telemetry.addData("Cells after prune", prunedCells.size());
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Recompute after start (to account for any last-second CSV changes)
        List<Cell> path = aStar(grid, new Cell(START_CX, START_CY), new Cell(GOAL_CX, GOAL_CY));
        if (path == null || path.isEmpty()) {
            telemetry.addLine("No path found (post-start).");
            telemetry.update();
            return;
        }
        List<Cell> pruned = prunePathWithLoS(grid, path);
        List<Pose2d> waypoints = toWaypointList(pruned);

        // ---- FOLLOW PATH ----
        for (Pose2d wp : waypoints) {
            goToWaypoint(wp);
            if (!opModeIsActive()) break;
        }

        drive(0,0,0);
        sleep(250);
    }

    // ------------------ PATH BUILDING ------------------

    /** A* on 8-connected grid with no corner-cutting through blockers. */
    List<Cell> aStar(int[][] g, Cell start, Cell goal) {
        int w = g[0].length, h = g.length;
        boolean[][] closed = new boolean[h][w];
        Cell[][] parent = new Cell[h][w];
        double[][] gScore = new double[h][w];
        for (double[] row : gScore) Arrays.fill(row, Double.POSITIVE_INFINITY);

        PriorityQueue<Cell> open = new PriorityQueue<>(Comparator.comparingDouble(c -> c.f));
        start.g = 0;
        start.f = start.g + hCost(start, goal);
        gScore[start.y][start.x] = 0;
        open.add(start);

        int[][] dirs = {
                { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
                { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
        };

        while (!open.isEmpty()) {
            Cell cur = open.poll();
            if (cur.x == goal.x && cur.y == goal.y) {
                return reconstruct(parent, cur);
            }
            if (closed[cur.y][cur.x]) continue;
            closed[cur.y][cur.x] = true;

            for (int[] d : dirs) {
                int nx = cur.x + d[0];
                int ny = cur.y + d[1];
                if (!inBounds(nx, ny, w, h)) continue;
                if (isBlocked(g, nx, ny)) continue;

                // prevent corner cutting: if moving diagonally, both cardinal neighbors must be free
                if (d[0] != 0 && d[1] != 0) {
                    if (isBlocked(g, cur.x + d[0], cur.y) || isBlocked(g, cur.x, cur.y + d[1])) {
                        continue;
                    }
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

    // Line-of-sight pruning using grid Bresenham (treat any blocked cell as obstacle)
    List<Cell> prunePathWithLoS(int[][] g, List<Cell> path) {
        if (path.size() <= 2) return path;
        ArrayList<Cell> pruned = new ArrayList<>();
        int i = 0;
        pruned.add(path.get(0));
        int j = 1;
        while (j < path.size()) {
            // Try to skip intermediate nodes by checking LoS from i -> j
            if (hasLineOfSight(g, path.get(i), path.get(j))) {
                // can skip nodes in between; advance j
                j++;
            } else {
                // last visible point is j-1
                pruned.add(path.get(j - 1));
                i = j - 1;
            }
        }
        // add final goal
        pruned.add(path.get(path.size() - 1));
        // optional: coalesce duplicates
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
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return pts;
    }

    List<Pose2d> toWaypointList(List<Cell> cells) {
        ArrayList<Pose2d> list = new ArrayList<>();
        for (Cell c : cells) {
            list.add(new Pose2d((c.x + 0.5) * CELL_IN, (c.y + 0.5) * CELL_IN, 0.0));
        }
        // Optional: add gentle heading toward next point for smoother turns
        for (int i = 0; i < list.size() - 1; i++) {
            Pose2d a = list.get(i), b = list.get(i + 1);
            double hdg = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
            a.h = hdg;
        }
        // Final pose faces upfield (0° already okay), or aim straight at back wall if you prefer:
        if (!list.isEmpty()) {
            Pose2d last = list.get(list.size() - 1);
            last.h = 0.0;
        }
        return list;
    }

    // ------------------ DRIVE / LOCALIZATION ------------------

    void goToWaypoint(Pose2d target) {
        while (opModeIsActive()) {
            updateLocalPoseFromEncodersAndIMU();

            double ex = target.x - pose.x;
            double ey = target.y - pose.y;
            double ePos = Math.hypot(ex, ey);

            double heading = normalizeDeg(pose.h);
            double eH = normalizeDeg(target.h - heading);

            double rad = Math.toRadians(heading);
            double cos = Math.cos(rad), sin = Math.sin(rad);
            double rx =  cos * ex + sin * ey;   // forward
            double ry = -sin * ex + cos * ey;   // left

            double vx = VX_kP * rx;
            double vy = VY_kP * ry;
            double om = HEADING_kP * eH;
            om = clamp(om, -MAX_TURN_POWER, MAX_TURN_POWER);

            if (ePos < POS_TOL_IN && Math.abs(eH) < HEADING_TOL_DEG) {
                drive(0,0,0);
                break;
            }
            drive(vx, vy * LATERAL_MULTIPLIER, om);

            telemetry.addData("Target (in)", "(%.1f, %.1f, %.1f°)", target.x, target.y, target.h);
            telemetry.addData("Pose   (in)", "(%.1f, %.1f, %.1f°)", pose.x, pose.y, pose.h);
            telemetry.update();
        }
    }

    private void drive(double vx, double vy, double omega) {
        double flP = vx + vy + omega;
        double frP = vx - vy - omega;
        double blP = vx - vy + omega;
        double brP = vx + vy - omega;
        double max = Math.max(1.0, Math.max(Math.abs(flP),
                Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));
        flP /= max; frP /= max; blP /= max; brP /= max;
        flP *= MAX_DRIVE_POWER; frP *= MAX_DRIVE_POWER; blP *= MAX_DRIVE_POWER; brP *= MAX_DRIVE_POWER;
        fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);
    }

    private void updateLocalPoseFromEncodersAndIMU() {
        int curFL = fl.getCurrentPosition();
        int curFR = fr.getCurrentPosition();
        int curBL = bl.getCurrentPosition();
        int curBR = br.getCurrentPosition();

        int dFL = curFL - lastFL;
        int dFR = curFR - lastFR;
        int dBL = curBL - lastBL;
        int dBR = curBR - lastBR;

        lastFL = curFL; lastFR = curFR; lastBL = curBL; lastBR = curBR;

        double ticksToIn = (2.0 * Math.PI * WHEEL_RADIUS_IN) * GEAR_RATIO / TICKS_PER_REV;
        double dFLin = dFL * ticksToIn;
        double dFRin = dFR * ticksToIn;
        double dBLin = dBL * ticksToIn;
        double dBRin = dBR * ticksToIn;

        double dXr = (dFLin + dFRin + dBLin + dBRin) / 4.0;
        double dYr = (-dFLin + dFRin + dBLin - dBRin) / 4.0 * LATERAL_MULTIPLIER;

        double imuHeading = -imu.getAngularOrientation().firstAngle; // adjust sign if needed

        double rad = Math.toRadians(imuHeading);
        double cos = Math.cos(rad), sin = Math.sin(rad);
        double dXf =  cos * dXr - sin * dYr;
        double dYf =  sin * dXr + cos * dYr;

        pose = new Pose2d(pose.x + dXf, pose.y + dYf, imuHeading);
    }

    // ------------------ GRID HELPERS ------------------

    void initDefaultGrid(int[][] g) {
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                g[y][x] = 0;
            }
        }
        // Perimeter walls (treat as blocked)
        for (int i = 0; i < GRID; i++) {
            g[0][i] = 1; g[GRID-1][i] = 1; g[i][0] = 1; g[i][GRID-1] = 1;
        }
    }

    void tryLoadCsvGrid(String path, int[][] g) {
        try {
            File f = new File(path);
            if (!f.exists()) return;
            BufferedReader br = new BufferedReader(new FileReader(f));
            ArrayList<String[]> rows = new ArrayList<>();
            String line;
            while ((line = br.readLine()) != null) {
                rows.add(line.split(","));
            }
            br.close();
            if (rows.size() != GRID || rows.get(0).length != GRID) return;

            // Interpret: dot "." => free(0). Anything else => blocked(1)
            for (int y = 0; y < GRID; y++) {
                for (int x = 0; x < GRID; x++) {
                    String token = rows.get(y)[x].trim();
                    g[y][x] = ".".equals(token) ? 0 : 1;
                }
            }
        } catch (Exception ignored) {}
    }

    boolean inBounds(int x, int y, int w, int h) {
        return x >= 0 && y >= 0 && x < w && y < h;
    }

    boolean isBlocked(int[][] g, int x, int y) {
        return g[y][x] >= 1;
    }

    double hCost(Cell a, Cell b) { return hCost(a.x, a.y, b.x, b.y); }
    double hCost(int x, int y, int gx, int gy) {
        // Octile heuristic
        int dx = Math.abs(x - gx), dy = Math.abs(y - gy);
        int diag = Math.min(dx, dy);
        int straight = Math.max(dx, dy) - diag;
        return diag * Math.sqrt(2) + straight;
    }

    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    static double normalizeDeg(double a) { while (a<=-180) a+=360; while (a>180) a-=360; return a; }

    // ------------------ TYPES ------------------

    static class Cell {
        int x, y;
        double g, f;
        Cell(int x, int y){ this(x,y,0,0); }
        Cell(int x, int y, double g, double f){ this.x=x; this.y=y; this.g=g; this.f=f; }
    }

    static class Pose2d {
        double x, y, h;  // inches, inches, degrees
        Pose2d(double x, double y, double h){ this.x=x; this.y=y; this.h=h; }
    }
}
