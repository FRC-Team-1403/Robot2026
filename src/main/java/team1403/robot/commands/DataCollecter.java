package team1403.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.Constants;
import team1403.robot.subsystems.Indexer;
import team1403.robot.subsystems.Shooter;
import team1403.robot.subsystems.ShooterHood;
import team1403.robot.subsystems.Spindexer;
import team1403.robot.subsystems.Turret;
import team1403.robot.util.Blackbox;
import team1403.robot.util.FieldZoneUtil;
import team1403.robot.util.FieldZoneUtil.Zone;

public class DataCollecter extends Command {
    private final Turret m_turret;
    private final Indexer m_indexer;
    private final Spindexer m_spindexer;
    private final Shooter m_shooter;
    private final ShooterHood m_shooterHood;
    private final Supplier<Pose2d> m_pose;
    private final DoubleSupplier m_shoot;
    private final BooleanSupplier m_increaseRPM;
    private final BooleanSupplier m_decreaseRPM;
    private final BooleanSupplier m_increaseHood;
    private final BooleanSupplier m_decreaseHood;
    private boolean isShooting;
    private boolean wasShooting;
    private Timer backupTimer;
    private double currentRPM;
    private double currentHood;
    private DigitalInput m_beamBreak1;
    private DigitalInput m_beamBreak2;
    private Timer m_ToFTimer;
    private double currentDistance;


//ONLY USE WITH 1 ball at a time otherwise ToF values will be completly wrong
    public DataCollecter(
            Turret turret,
            Indexer indexer,
            Spindexer spindexer,
            Shooter shooter,
            ShooterHood hood,
            Supplier<Pose2d> pose,
            DoubleSupplier shoot,
            BooleanSupplier increaseRPM,
            BooleanSupplier decreaseRPM,
            BooleanSupplier increaseHood,
            BooleanSupplier decreaseHood) {
        m_turret = turret;
        m_indexer = indexer;
        m_spindexer = spindexer;
        m_shooter = shooter;
        m_shooterHood = hood;
        m_pose = pose;
        m_shoot = shoot;
        m_increaseRPM = increaseRPM;
        m_decreaseRPM = decreaseRPM;
        m_increaseHood = increaseHood;
        m_decreaseHood = decreaseHood;
        isShooting = false;
        wasShooting = false;
        backupTimer = new Timer();
        currentRPM = 1000;
        currentHood = 5;
        m_beamBreak1 = new DigitalInput(0);
        m_beamBreak2 = new DigitalInput(0);
        m_ToFTimer = new Timer();
        currentDistance = 0;
        addRequirements(indexer, spindexer, shooter, hood, turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        if (m_beamBreak1.get()) {
            m_ToFTimer.reset();
            m_ToFTimer.start();
        }

        //Get updated values
        Pose2d robotPose = m_pose.get();
        boolean humanInput = m_shoot.getAsDouble() > 0.3;
        boolean increaseRPM = m_increaseRPM.getAsBoolean();
        boolean decreaseRPM = m_decreaseRPM.getAsBoolean();
        boolean increaseHood = m_increaseHood.getAsBoolean();
        boolean decreaseHood = m_decreaseHood.getAsBoolean();

        //Transform robot into turret
        Translation2d turretPivotField = robotPose.getTranslation()
                .plus(Constants.Turret.kTurretOffset.rotateBy(robotPose.getRotation()));

        //Calculate current values
        Translation2d target = Blackbox.getActiveTarget(robotPose);
        double deltaX = target.getX() - turretPivotField.getX();
        double deltaY = target.getY() - turretPivotField.getY();
        double distance = Math.hypot(deltaX, deltaY);
        currentDistance = distance;

        //Find and set turret
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));
        double robotHeading = robotPose.getRotation().getDegrees();
        double turretAngle = MathUtil.inputModulus(
                fieldAngleToGoal - robotHeading - Constants.Turret.rotationCorrectionOffset,
                Constants.Turret.kMinAngleDegrees,
                Constants.Turret.kMaxAngleDegrees);
        m_turret.setSetpoint(turretAngle);

        if (increaseRPM) {
            currentRPM += 10;
        }

        if (decreaseRPM) {
            currentRPM -= 10;
        }

        if (increaseHood) {
            currentHood += 0.01;
        }

        if (decreaseHood) {
            currentHood -= 0.01;
        }

        //If shoot button is pressed
        if (humanInput) {
            //Set flywheel
            m_shooter.setFlywheelTargetRPM(currentRPM);

            //Set hood
            Zone zone = FieldZoneUtil.getZone(robotPose);
            if (zone == Zone.CROSSING) {
                m_shooterHood.setSetpoint(0);
            }
            else {
                m_shooterHood.setSetpoint(currentHood);
            }
        } 
        //Otherwise reset to safe conditions
        else {
            isShooting = false;
            m_shooter.setFlywheelTargetRPM(0);
            m_shooterHood.setSetpoint(0);

            //If currently backing up
            if (backupTimer.isRunning() && backupTimer.get() < Constants.Shooter.kBackupTime) {
                m_spindexer.setSpindexerRPM(-2000);
                m_indexer.setIndexerRPM(-1800);
            } else {
                backupTimer.stop();
                backupTimer.reset();
                m_spindexer.setSpindexerRPM(0);
                m_indexer.setIndexerRPM(0);
            }
        }

        boolean allowedToShoot = m_shooter.isFlywheelAtSpeed()
                                && m_shooterHood.atSetpoint()
                                && humanInput
                                && m_turret.atSetpoint();

        //Should we actually start shooting
        if (allowedToShoot) {
            isShooting = true;
            m_spindexer.setSpindexerRPM(Constants.Spindexer.m_spindexerRPM);
            m_indexer.setIndexerRPM(Constants.Indexer.m_indexerRPM);
        }

        //If shooting changed from true to false
        if (wasShooting && !isShooting) {
            backupTimer.reset();
            backupTimer.start();
        }

        //Update wasShooting
        wasShooting = isShooting;
    }

    //Should never run (Default Command)
    @Override
    public void end(boolean interrupted) {
        m_spindexer.stop();
        m_indexer.stop();
        m_shooter.stop();
        m_shooterHood.setSetpoint(0);

        m_ToFTimer.stop();
        System.out.println("Distance: " + currentDistance + " HoodAngle: " + currentHood + " RPM: " + currentRPM + " ToF: " + m_ToFTimer.get());
    }

    //Should never run (Default Command)
    @Override
    public boolean isFinished() {
        return m_beamBreak2.get();
    }
}