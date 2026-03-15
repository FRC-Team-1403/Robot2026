package team1403.robot.util;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.IOException;

/**
 * Loads the precomputed shoot-on-the-move lookup table from deploy/shooter_table.json
 * and provides trilinear-interpolated lookups from (dx, dy, vx, vy) → (rpm, hood, turret).
 */
public class ShooterTable {

  public static final class Result {
    public final double  rpm;
    public final double  hoodAngle;
    public final double  turretAngle;
    public final boolean valid;

    private Result(double rpm, double hoodAngle, double turretAngle, boolean valid) {
      this.rpm        = rpm;
      this.hoodAngle  = hoodAngle;
      this.turretAngle = turretAngle;
      this.valid      = valid;
    }
  }

  private final double distMin, distStep;
  private final double vxMin,   vxStep;
  private final double vyMin,   vyStep;
  private final int    distCount, vxCount, vyCount;

  private final double[] rpm;
  private final double[] hood;
  private final double[] turret;
  private final int[]    valid;

  private ShooterTable(
      double distMin, double distStep, int distCount,
      double vxMin,   double vxStep,   int vxCount,
      double vyMin,   double vyStep,   int vyCount,
      double[] rpm, double[] hood, double[] turret, int[] valid) {
    this.distMin = distMin; this.distStep = distStep; this.distCount = distCount;
    this.vxMin   = vxMin;   this.vxStep   = vxStep;   this.vxCount   = vxCount;
    this.vyMin   = vyMin;   this.vyStep   = vyStep;   this.vyCount   = vyCount;
    this.rpm = rpm; this.hood = hood; this.turret = turret; this.valid = valid;
  }

  // ── Loader ─────────────────────────────────────────────────────────────────

  /**
   * Streaming JSON load — reads values directly without building a tree.
   * Works on files of any size without blowing up the heap.
   */
  public static ShooterTable load() throws IOException {
    File f = new File(Filesystem.getDeployDirectory(), "shooter_table.json");
    System.out.println("[ShooterTable] Loading from: " + f.getAbsolutePath());

    // Meta fields — populated while streaming the "meta" object
    double distMin = 0, distMax = 0, distStep = 0; int distCount = 0;
    double vxMin   = 0, vxMax   = 0, vxStep   = 0; int vxCount   = 0;
    double vyMin   = 0, vyMax   = 0, vyStep   = 0; int vyCount   = 0;

    double[] rpmArr = null, hoodArr = null, turretArr = null;
    int[]    validArr = null;

    JsonFactory factory = new JsonFactory();
    try (JsonParser p = factory.createParser(f)) {

      // Expect opening '{'
      p.nextToken(); // START_OBJECT

      while (p.nextToken() != JsonToken.END_OBJECT) {
        String section = p.currentName();
        p.nextToken(); // move into value

        switch (section) {

          case "meta": {
            // Stream through the meta object key by key
            while (p.nextToken() != JsonToken.END_OBJECT) {
              String key = p.currentName();
              p.nextToken();
              switch (key) {
                case "distMin":   distMin   = p.getDoubleValue(); break;
                case "distMax":   distMax   = p.getDoubleValue(); break;
                case "distStep":  distStep  = p.getDoubleValue(); break;
                case "distCount": distCount = p.getIntValue();    break;
                case "vxMin":     vxMin     = p.getDoubleValue(); break;
                case "vxMax":     vxMax     = p.getDoubleValue(); break;
                case "vxStep":    vxStep    = p.getDoubleValue(); break;
                case "vxCount":   vxCount   = p.getIntValue();    break;
                case "vyMin":     vyMin     = p.getDoubleValue(); break;
                case "vyMax":     vyMax     = p.getDoubleValue(); break;
                case "vyStep":    vyStep    = p.getDoubleValue(); break;
                case "vyCount":   vyCount   = p.getIntValue();    break;
                default: p.skipChildren(); break;
              }
            }
            int total = distCount * vxCount * vyCount;
            rpmArr    = new double[total];
            hoodArr   = new double[total];
            turretArr = new double[total];
            validArr  = new int[total];
            System.out.println("[ShooterTable] Meta parsed. Total cells: " + total);
            break;
          }

          case "rpm":    readDoubleArray(p, rpmArr);    break;
          case "hood":   readDoubleArray(p, hoodArr);   break;
          case "turret": readDoubleArray(p, turretArr); break;
          case "valid":  readIntArray(p,    validArr);  break;
          default:       p.skipChildren();               break;
        }
      }
    }

    System.out.println("[ShooterTable] Load complete.");
    return new ShooterTable(
        distMin, distStep, distCount,
        vxMin,   vxStep,   vxCount,
        vyMin,   vyStep,   vyCount,
        rpmArr, hoodArr, turretArr, validArr);
  }

  private static void readDoubleArray(JsonParser p, double[] out) throws IOException {
    int i = 0;
    while (p.nextToken() != JsonToken.END_ARRAY)
      out[i++] = p.getDoubleValue();
  }

  private static void readIntArray(JsonParser p, int[] out) throws IOException {
    int i = 0;
    while (p.nextToken() != JsonToken.END_ARRAY)
      out[i++] = p.getIntValue();
  }

  // ── Lookup ─────────────────────────────────────────────────────────────────

  public Result lookup(double dx, double dy, double vx, double vy) {
    double dist = Math.sqrt(dx * dx + dy * dy);

    dist = Math.max(distMin, Math.min(dist, distMin + distStep * (distCount - 1)));
    vx   = Math.max(vxMin,   Math.min(vx,   vxMin   + vxStep   * (vxCount   - 1)));
    vy   = Math.max(vyMin,   Math.min(vy,   vyMin   + vyStep   * (vyCount   - 1)));

    double di_f = (dist - distMin) / distStep;
    double xi_f = (vx   - vxMin)   / vxStep;
    double yi_f = (vy   - vyMin)   / vyStep;

    int di0 = (int) di_f,  di1 = Math.min(di0 + 1, distCount - 1);
    int xi0 = (int) xi_f,  xi1 = Math.min(xi0 + 1, vxCount   - 1);
    int yi0 = (int) yi_f,  yi1 = Math.min(yi0 + 1, vyCount   - 1);

    double td = di_f - di0, tx = xi_f - xi0, ty = yi_f - yi0;

    double rpmVal    = trilinear(rpm,    di0, di1, xi0, xi1, yi0, yi1, td, tx, ty);
    double hoodVal   = trilinear(hood,   di0, di1, xi0, xi1, yi0, yi1, td, tx, ty);
    double turretVal = trilinear(turret, di0, di1, xi0, xi1, yi0, yi1, td, tx, ty);
    boolean validVal = validCorners(di0, di1, xi0, xi1, yi0, yi1);

    double geometricTurret = Math.toDegrees(Math.atan2(dy, dx));
    return new Result(rpmVal, hoodVal, turretVal + geometricTurret, validVal);
  }

  // ── Helpers ────────────────────────────────────────────────────────────────

  private int idx(int di, int xi, int yi) {
    return di * (vxCount * vyCount) + xi * vyCount + yi;
  }

  private double trilinear(double[] a,
      int di0, int di1, int xi0, int xi1, int yi0, int yi1,
      double td, double tx, double ty) {
    double c00 = lerp(a[idx(di0,xi0,yi0)], a[idx(di0,xi0,yi1)], ty);
    double c01 = lerp(a[idx(di0,xi1,yi0)], a[idx(di0,xi1,yi1)], ty);
    double c10 = lerp(a[idx(di1,xi0,yi0)], a[idx(di1,xi0,yi1)], ty);
    double c11 = lerp(a[idx(di1,xi1,yi0)], a[idx(di1,xi1,yi1)], ty);
    return lerp(lerp(c00, c01, tx), lerp(c10, c11, tx), td);
  }

  private boolean validCorners(int di0, int di1, int xi0, int xi1, int yi0, int yi1) {
    return valid[idx(di0,xi0,yi0)] == 1 && valid[idx(di0,xi0,yi1)] == 1
        && valid[idx(di0,xi1,yi0)] == 1 && valid[idx(di0,xi1,yi1)] == 1
        && valid[idx(di1,xi0,yi0)] == 1 && valid[idx(di1,xi0,yi1)] == 1
        && valid[idx(di1,xi1,yi0)] == 1 && valid[idx(di1,xi1,yi1)] == 1;
  }

  private static double lerp(double a, double b, double t) { return a + t * (b - a); }
}
