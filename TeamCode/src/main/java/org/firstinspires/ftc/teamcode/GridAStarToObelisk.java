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
 * ---------------------------------------------------------------------------
 * GRID A* AUTONOMOUS – 48x48 map with 3" cells + mecanum follower
 * ---------------------------------------------------------------------------
 * PURPOSE
 *   Plan a collision-free path on a 48x48 grid (3" per cell), turn that path
 *   into waypoints (inches), then drive a mecanum robot through those waypoints.
 *
 * COORDINATE SYSTEM (VERY IMPORTANT)
 *   - Field size: 144" x 144" (FTC field).
 *   - Grid cells: 48 x 48, each cell = 3" x 3".
 *   - Origin (0,0): audience-left / front INSIDE the perimeter.
 *   - +X: to the right (toward tile F), +Y: away from audience (toward row 6).
 *   - The CENTER of any cell (cx,cy) is ((cx+0.5)*3, (cy+0.5)*3) inches.
 *
 * WHAT THIS OP MODE DOES (step by step)
 *   1) Build a map: start with empty interior; mark perimeter as blocked.
 *      Optionally load /sdcard/FIRST/ftc_field_grid_3in.csv to override obstacles.
 *   2) Run A* from START cell to GOAL cell to get a list of grid cells.
 *   3) Prune the list with line-of-sight to reduce zig-zags (fewer waypoints).
 *   4) Convert those grid cells to field inches (waypoints).
 *   5) Drive to each waypoint using simple P-control on x/y + IMU for heading.
 *
 * HOW TO CHANGE THE ROUTE
 *   - Edit START_CX/START_CY and GOAL_CX/GOAL_CY below, OR
 *   - Load a CSV map that blocks off areas to “shape” the A* path, OR
 *   - Use the desktop Course Plotter to export JSON/Java waypoints.
 *
 * STUDENT EXERCISES / KNOBS
 *   - Tune LATERAL_MULTIPLIER so strafes measure correctly.
 *   - Tune kP gains to make the robot smooth but responsive.
 *   - Add “inflation” (grow obstacles) to respect robot width.
 *   - Replace waypoint follower with a Ramsete/PID/holonomic controller later.
 */
@Autonomous(name = "Grid A* to Obelisk (EXTRA COMMENTED)", group = "Competition")
public class GridAStarToObelisk extends LinearOpMode {

    // ---------------- GRID / SCALE ----------------
    // 48x48 cells * 3 inches per cell = 144" field.
    static final int GRID = 48;
    static final double CELL_IN = 3.0;

    // ---------------- ROBOT CONSTANTS (TUNE THESE) ----------------
    // Motor encoder parameters and geometry of your drivetrain.
    public static final double TICKS_PER_REV      = 537.7;  // e.g., goBILDA 5202/003
    public static final double WHEEL_RADIUS_IN    = 1.889;  // half of wheel diameter (~3.78") – CHANGE if different
    public static final double GEAR_RATIO         = 1.0;    // if wheels are geared up/down from motor
    public static final double TRACK_WIDTH_IN     = 14.0;   // left-right wheel separation (approx – not critical here)
    public static final double LATERAL_MULTIPLIER = 1.08;   // strafing scaling (accounts for mechanical losses)

    // Simple proportional gains for driving to a waypoint.
    public static final double VX_kP = 0.085;     // forward speed per inch of error
    public static final double VY_kP = 0.085;     // strafe speed per inch of error
    public static final double HEADING_kP = 0.015;// turn speed per degree of error
    public static final double MAX_DRIVE_POWER = 0.7;  // safety caps
    public static final double MAX_TURN_POWER  = 0.5;
    public static final double POS_TOL_IN      = 2.0;  // stop when within 2" of waypoint
    public static final double HEADING_TOL_DEG = 7.0;  // and within 7° of desired heading

    // ---------------- START / GOAL (CELLS) ----------------
    // Choose start and goal in CELL coordinates. We pick the Blue front-right corner-ish
    // to an approach point just in front of the Obelisk (back wall center).
    static final int START_CX = 46, START_CY = 2;  // adjust to your exact start
    static final int GOAL_CX  = 24, GOAL_CY  = 46; // one cell (~3") off back wall near middle

    // ---------------- HARDWARE ----------------
    private DcMotor fl, fr, bl, br;     // frontLeft, frontRight, backLeft, backRight
    private BNO055IMU imu;              // gyroscope for heading (yaw)

    // Localization state (very simple: encoders for x/y; IMU for heading)
    private int lastFL, lastFR, lastBL, lastBR;
    private Pose2d pose = new Pose2d(
            (START_CX + 0.5) * CELL_IN, // start X inches
            (START_CY + 0.5) * CELL_IN, // start Y inches
            0.0                         // start facing "upfield" (toward +Y)
    );

    // The cost map: 0 = free, >=1 = blocked (we treat any >=1 as an obstacle)
    private int[][] grid = new int[GRID][GRID];

    @Override
    public void runOpMode() {
        // 1) BUILD THE MAP (perimeter walls blocked; try to load CSV if present)
        initDefaultGrid(grid); // perimeter blocked; interior free
        tryLoadCsvGrid("/sdcard/FIRST/ftc_field_grid_3in.csv", grid); // optional map from phone

        // 2) MAP HARDWARE (RENAME MOTOR NAMES TO MATCH YOUR CONFIG!)
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        fr = hardwareMap.get(DcMotor.class, "frontRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");

        // Mecanum convention: left motors usually reversed. Verify in TeleOp.
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        // Use encoders as sensors (we control by power, not by target position)
        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // 3) GYRO/IMU for heading (degrees)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(p);

        // Capture encoder baselines
        lastFL = fl.getCurrentPosition();
        lastFR = fr.getCurrentPosition();
        lastBL = bl.getCurrentPosition();
        lastBR = br.getCurrentPosition();

        telemetry.addLine("A* planner ready");
        telemetry.addData("Start cell", "(%d,%d)", START_CX, START_CY);
        telemetry.addData("Goal cell",  "(%d,%d)", GOAL_CX,  GOAL_CY);
        telemetry.update();

        // 4) PLAN THE PATH (pre-plan so you can see if it's solvable before you press start)
        List<Cell> preview = aStar(grid, new Cell(START_CX, START_CY), new Cell(GOAL_CX, GOAL_CY));
        if (preview == null || preview.isEmpty()) {
            telemetry.addLine("No path found (pre-start). Check your map or cells.");
            telemetry.update();
        } else {
            List<Cell> prunedPreview = prunePathWithLoS(grid, preview);
            telemetry.addData("Path cells (raw)", preview.size());
            telemetry.addData("After prune", prunedPreview.size());
            telemetry.update();
        }

        // Wait for start button
        waitForStart();
        if (isStopRequested()) return;

        // 5) PLAN AGAIN (allows last-second CSV edits before start)
        List<Cell> path = aStar(grid, new Cell(START_CX, START_CY), new Cell(GOAL_CX, GOAL_CY));
        if (path == null || path.isEmpty()) {
            telemetry.addLine("No path found (post-start).");
            telemetry.update();
            return;
        }

        // Reduce corners by skipping points that have clear line-of-sight
        List<Cell> pruned = prunePathWithLoS(grid, path);

        // Convert to inch-based waypoints (center of each cell) and assign headings
        List<Pose2d> waypoints = toWaypointList(pruned);

        // 6) FOLLOW THE WAYPOINTS UNTIL DONE (or STOP pressed)
        for (Pose2d wp : waypoints) {
            goToWaypoint(wp);
            if (!opModeIsActive()) break;
        }

        // Always stop motors at the end
        drive(0,0,0);
        sleep(200);
    }

    // ------------------------- PATH PLANNING -------------------------

    /**
     * A* search on a grid (8-connected). We allow diagonals BUT prevent
     * "corner cutting" (can't skim diagonally between two touching obstacles).
     */
    List<Cell> aStar(int[][] g, Cell start, Cell goal) {
        int w = g[0].length, h = g.length;

        // Bookkeeping structures
        boolean[][] closed = new boolean[h][w];   // explored set
        Cell[][] parent    = new Cell[h][w];      // for reconstructing the final path
        double[][] gScore  = new double[h][w];    // cost from start
        for (double[] row : gScore) Arrays.fill(row, Double.POSITIVE_INFINITY);

        // Priority queue ordered by total estimated cost f = g + h
        PriorityQueue<Cell> open = new PriorityQueue<>(Comparator.comparingDouble(c -> c.f));

        // Initialize start
        start.g = 0;
        start.f = start.g + hCost(start, goal);
        gScore[start.y][start.x] = 0;
        open.add(start);

        // 8 neighbor directions
        int[][] dirs = {
                { 1, 0},{-1, 0},{ 0, 1},{ 0,-1},
                { 1, 1},{ 1,-1},{-1, 1},{-1,-1}
        };

        while (!open.isEmpty()) {
            Cell cur = open.poll();

            // If already processed, skip
            if (closed[cur.y][cur.x]) continue;

            // Goal reached: rebuild by walking parents backward
            if (cur.x == goal.x && cur.y == goal.y) {
                return reconstruct(parent, cur);
            }

            closed[cur.y][cur.x] = true;

            // Explore neighbors
            for (int[] d : dirs) {
                int nx = cur.x + d[0];
                int ny = cur.y + d[1];

                // bounds + obstacle check
                if (!inBounds(nx, ny, w, h)) continue;
                if (isBlocked(g, nx, ny)) continue;

                // No corner-cutting around diagonals
                if (d[0] != 0 && d[1] != 0) {
                    if (isBlocked(g, cur.x + d[0], cur.y) || isBlocked(g, cur.x, cur.y + d[1])) {
                        continue;
                    }
                }

                // Moving straight = cost 1, diagonal = sqrt(2)
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
        // No path
        return null;
    }

    /** Turn the chain of parents into an ordered list from start -> goal. */
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

    /** Prune points that are redundant (straight line-of-sight through free cells). */
    List<Cell> prunePathWithLoS(int[][] g, List<Cell> path) {
        if (path.size() <= 2) return path;
        ArrayList<Cell> pruned = new ArrayList<>();
        int i = 0;
        pruned.add(path.get(0));
        int j = 1;
        while (j < path.size()) {
            // If we can see from i -> j directly, keep extending j
            if (hasLineOfSight(g, path.get(i), path.get(j))) {
                j++;
            } else {
                // last visible is j-1
                pruned.add(path.get(j - 1));
                i = j - 1;
            }
        }
        pruned.add(path.get(path.size() - 1));

        // Remove accidental duplicates
        ArrayList<Cell> cleaned = new ArrayList<>();
        Cell last = null;
        for (Cell c : pruned) {
            if (last == null || !(last.x == c.x && last.y == c.y)) cleaned.add(c);
            last = c;
        }
        return cleaned;
    }

    /** Bresenham grid line-of-sight check: every cell on the line must be free. */
    boolean hasLineOfSight(int[][] g, Cell a, Cell b) {
        for (int[] p : bresenham(a.x, a.y, b.x, b.y)) {
            if (isBlocked(g, p[0], p[1])) return false;
        }
        return true;
    }

    /** Discrete line drawing algorithm; returns all grid cells touched by the segment. */
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

    /** Convert a list of cells to inch waypoints and assign heading toward next point. */
    List<Pose2d> toWaypointList(List<Cell> cells) {
        ArrayList<Pose2d> list = new ArrayList<>();
        for (Cell c : cells) {
            list.add(new Pose2d((c.x + 0.5) * CELL_IN, (c.y + 0.5) * CELL_IN, 0.0));
        }
        // Face each waypoint toward the next one to reduce spin-in-place behavior
        for (int i = 0; i < list.size() - 1; i++) {
            Pose2d a = list.get(i), b = list.get(i + 1);
            double hdg = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
            a.h = hdg;
        }
        // Final pose: face upfield (or set to 0°); adjust if you want to face an object
        if (!list.isEmpty()) {
            list.get(list.size() - 1).h = 0.0;
        }
        return list;
    }

    // ------------------------- DRIVING / LOCALIZATION -------------------------

    /**
     * Drive until we are within position + heading tolerance of the target pose.
     * Uses very simple proportional control: speed ~ error.
     */
    void goToWaypoint(Pose2d target) {
        while (opModeIsActive()) {
            // Update our best guess of (x,y,heading)
            updateLocalPoseFromEncodersAndIMU();

            // Errors in FIELD frame (inches and degrees)
            double ex = target.x - pose.x;
            double ey = target.y - pose.y;
            double ePos = Math.hypot(ex, ey);        // distance to waypoint
            double heading = normalizeDeg(pose.h);    // current yaw (deg)
            double eH = normalizeDeg(target.h - heading);

            // Convert field errors to ROBOT frame (so forward = x, left = y)
            double rad = Math.toRadians(heading);
            double cos = Math.cos(rad), sin = Math.sin(rad);
            double rx =  cos * ex + sin * ey;   // forward error
            double ry = -sin * ex + cos * ey;   // left error (strafe)

            // Proportional commands
            double vx = VX_kP * rx;
            double vy = VY_kP * ry;
            double om = HEADING_kP * eH;
            om = clamp(om, -MAX_TURN_POWER, MAX_TURN_POWER);

            // Stop if we are "close enough"
            if (ePos < POS_TOL_IN && Math.abs(eH) < HEADING_TOL_DEG) {
                drive(0,0,0);
                break;
            }

            // Drive (note: scale strafe by LATERAL_MULTIPLIER to correct distance)
            drive(vx, vy * LATERAL_MULTIPLIER, om);

            // Debug info on Driver Station
            telemetry.addData("Target (in)", "(%.1f, %.1f, %.1f°)", target.x, target.y, target.h);
            telemetry.addData("Pose   (in)", "(%.1f, %.1f, %.1f°)", pose.x, pose.y, pose.h);
            telemetry.update();
        }
    }

    /**
     * Mecanum power mapping: given desired robot-centric velocities (vx forward,
     * vy left, omega CCW), compute motor powers and cap them safely.
     */
    private void drive(double vx, double vy, double omega) {
        double flP = vx + vy + omega;
        double frP = vx - vy - omega;
        double blP = vx - vy + omega;
        double brP = vx + vy - omega;

        // Normalize so the largest magnitude is 1.0
        double max = Math.max(1.0, Math.max(Math.abs(flP),
                Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));
        flP /= max; frP /= max; blP /= max; brP /= max;

        // Global cap for safety
        flP *= MAX_DRIVE_POWER; frP *= MAX_DRIVE_POWER;
        blP *= MAX_DRIVE_POWER; brP *= MAX_DRIVE_POWER;

        fl.setPower(flP); fr.setPower(frP); bl.setPower(blP); br.setPower(brP);
    }

    /**
     * Very lightweight localization:
     *  - Use encoders to estimate robot-frame delta (forward/strafe).
     *  - Use IMU yaw for heading (more reliable than “rotation from wheel deltas” on mecanum).
     *  - Rotate robot-frame delta into the field frame and accumulate.
     */
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

        // Convert encoder ticks to inches traveled by each wheel
        double ticksToIn = (2.0 * Math.PI * WHEEL_RADIUS_IN) * GEAR_RATIO / TICKS_PER_REV;
        double dFLin = dFL * ticksToIn;
        double dFRin = dFR * ticksToIn;
        double dBLin = dBL * ticksToIn;
        double dBRin = dBR * ticksToIn;

        // Approx mecanum kinematics for small motions
        double dXr = (dFLin + dFRin + dBLin + dBRin) / 4.0;                        // forward
        double dYr = (-dFLin + dFRin + dBLin - dBRin) / 4.0 * LATERAL_MULTIPLIER;   // strafe

        // Heading from IMU. If sign is wrong, flip it here.
        double imuHeading = -imu.getAngularOrientation().firstAngle;

        // Rotate robot delta into field frame using current heading
        double rad = Math.toRadians(imuHeading);
        double cos = Math.cos(rad), sin = Math.sin(rad);
        double dXf =  cos * dXr - sin * dYr;
        double dYf =  sin * dXr + cos * dYr;

        pose = new Pose2d(pose.x + dXf, pose.y + dYf, imuHeading);
    }

    // ------------------------- MAP / UTILS -------------------------

    /** Start with empty interior and blocked perimeter. */
    void initDefaultGrid(int[][] g) {
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                g[y][x] = 0; // free
            }
        }
        // perimeter walls
        for (int i = 0; i < GRID; i++) {
            g[0][i] = 1; g[GRID-1][i] = 1; g[i][0] = 1; g[i][GRID-1] = 1;
        }
    }

    /**
     * Optional: load a 48x48 CSV from phone storage.
     * '.' means free; any other token means blocked.
     * We always keep the perimeter blocked for safety.
     */
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

            for (int y = 0; y < GRID; y++) {
                for (int x = 0; x < GRID; x++) {
                    if (x == 0 || x == GRID-1 || y == 0 || y == GRID-1) {
                        g[y][x] = 1; // perimeter forced blocked
                    } else {
                        String token = rows.get(y)[x].trim();
                        g[y][x] = ".".equals(token) ? 0 : 1;
                    }
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

    // Octile heuristic (perfect for 8-connected grids)
    double hCost(Cell a, Cell b) { return hCost(a.x, a.y, b.x, b.y); }
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
        while (a >   180) a -= 360;
        return a;
    }

    // ------------------------- SMALL TYPES -------------------------

    /** Grid cell with optional g/f costs for A*. */
    static class Cell {
        int x, y;
        double g, f;  // path cost so far & total estimated cost
        Cell(int x, int y){ this(x,y,0,0); }
        Cell(int x, int y, double g, double f){ this.x=x; this.y=y; this.g=g; this.f=f; }
    }

    /** Simple 2D pose in inches & degrees. */
    static class Pose2d {
        double x, y, h;
        Pose2d(double x, double y, double h){ this.x=x; this.y=y; this.h=h; }
    }
}
