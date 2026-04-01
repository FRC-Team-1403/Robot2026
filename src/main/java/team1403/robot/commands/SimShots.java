package team1403.robot.commands;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimShots {

    private static class Shot {
        Translation3d pos;
        Translation3d vel;
        double time;

        Shot(Translation3d pos, Translation3d vel) {
            this.pos = pos;
            this.vel = vel;
            this.time = 0;
        }

        void update(double dt) {
            double g = -9.81;

            // Update velocity
            vel = new Translation3d(
                vel.getX(),
                vel.getY(),
                vel.getZ() + g * dt
            );

            // Update position
            pos = new Translation3d(
                pos.getX() + vel.getX() * dt,
                pos.getY() + vel.getY() * dt,
                pos.getZ() + vel.getZ() * dt
            );

            time += dt;
        }

        Pose3d getPose() {
            return new Pose3d(pos, new Rotation3d());
        }
    }

    private static final List<Shot> shots = new ArrayList<>();

    // ====== CONFIG ======
    private static final double SHOOTER_HEIGHT = 0.7; // meters (tune)
    private static final double RPM_TO_MPS = 0.0035;   // tune
    private static final double MAX_LIFETIME = 3.0;   // seconds

    // ====== HOOD -> PITCH (your fitted line) ======
    private static double hoodToPitchDeg(double hoodDeg) {
        return -2.0 * hoodDeg + 90.0;
    }

    // ====== FIRE ======
    public static void fire(
            Translation2d turretPos,
            double turretAngleDeg,
            double hoodAngleDeg,
            double flywheelRPM,
            Rotation2d robotHeading,
            ChassisSpeeds robotVelocity
    ) {
        // Convert angles
        double turretWorldRad = Math.toRadians(turretAngleDeg) + robotHeading.getRadians();
        double pitchRad = Math.toRadians(hoodToPitchDeg(hoodAngleDeg));

        // RPM -> linear velocity
        double speed = flywheelRPM * RPM_TO_MPS;

        // Velocity components
        double vx = speed * Math.cos(pitchRad) * Math.cos(turretWorldRad);
        double vy = speed * Math.cos(pitchRad) * Math.sin(turretWorldRad);
        double vz = speed * Math.sin(pitchRad);

        // Add robot velocity (shoot-on-the-fly)
        vx += robotVelocity.vxMetersPerSecond;
        vy += robotVelocity.vyMetersPerSecond;

        Translation3d startPos = new Translation3d(
            turretPos.getX(),
            turretPos.getY(),
            SHOOTER_HEIGHT
        );

        Translation3d startVel = new Translation3d(vx, vy, vz);

        shots.add(new Shot(startPos, startVel));
    }

    // ====== UPDATE ======
    public static void updateAll(double dt) {
        Iterator<Shot> it = shots.iterator();

        while (it.hasNext()) {
            Shot s = it.next();
            s.update(dt);

            if (s.time > MAX_LIFETIME || s.pos.getZ() < 0) {
                it.remove();
            }
        }
    }

    // ====== OUTPUT ======
    public static Pose3d[] getPoses() {
        Pose3d[] poses = new Pose3d[shots.size()];
        for (int i = 0; i < shots.size(); i++) {
            poses[i] = shots.get(i).getPose();
        }
        return poses;
    }

    // ====== CLEAR ======
    public static void clear() {
        shots.clear();
    }
}