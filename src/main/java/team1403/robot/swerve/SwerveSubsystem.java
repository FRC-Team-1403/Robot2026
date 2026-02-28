package team1403.robot.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team1403.lib.util.CougarUtil;
import team1403.robot.Constants;
import team1403.robot.swerve.TunerConstants.TunerSwerveDrivetrain;
import team1403.robot.swerve.util.SwerveHeadingCorrector;
import team1403.robot.vision.ITagCamera;
import team1403.robot.vision.VisionSimUtil;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem, Sendable {
    private static final double kSimLoopPeriod = 0.005;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveHeadingCorrector m_headingCorrector = new SwerveHeadingCorrector();
    private final Alert m_gyroDisconnected = new Alert("Gyroscope Disconnected!", AlertType.kError);
    private Rotation2d m_headingOffset = Rotation2d.kZero;
    private SwerveDriveState m_state;

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private List<ITagCamera> m_cameras = new ArrayList<>();

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(Math.PI / 6).per(Second),
            Volts.of(Math.PI),
            null,
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    private Telemetry m_telemetry;

    public void setCameras(List<ITagCamera> cameras) {
        m_cameras = cameras;
    }    

    private void onConstruct() {
      super.resetPose(CougarUtil.getInitialRobotPose());

      AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        () -> m_state.Speeds,
        (s, ff) -> drive(s, ff),
        new PPHolonomicDriveController(
            TunerConstants.kTranslationPID,
            TunerConstants.kRotationPID,
            Constants.kLoopTime
        ),
        CougarUtil.loadRobotConfig(),
        () -> CougarUtil.shouldMirrorPath(),
        this);

        Pathfinding.setPathfinder(new LocalADStar());

        Commands.sequence(
            FollowPathCommand.warmupCommand(),
            PathfindingCommand.warmupCommand()
        ).schedule();

        SmartDashboard.putData("Gyro", super.getPigeon2());

        m_telemetry = new Telemetry(TunerConstants.kMaxSpeed);
        m_state = getState();
    }

    public SwerveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        onConstruct();
    }

    public SwerveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        onConstruct();
    }

    public SwerveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        onConstruct();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void resetOdometry() {
        resetOdometry(getPose());
    }

    public void resetOdometry(Pose2d pose) {
        this.resetPose(pose);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
        });
        }
        
        VisionSimUtil.update(getPose());

        for (ITagCamera camera : m_cameras) {
            if (camera.checkVisionResult()) {
                addVisionMeasurement(
                    camera.getPose().toPose2d(),
                    camera.getTimestamp(),
                    camera.getEstStdv()
                );
            }
        }

        m_state = getState();

        m_gyroDisconnected.set(!super.getPigeon2().isConnected());

        SmartDashboard.putNumber("Velocity", CougarUtil.norm(m_state.Speeds));

        m_telemetry.telemeterize(m_state);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        SwerveModuleState[] states = m_state.ModuleStates;

        builder.addDoubleProperty("Front Left Angle", () -> states[0].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> states[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> states[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> states[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> states[2].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> states[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> states[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> states[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getRotation().getRadians(), null);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose() {
        return m_state.Pose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Rotation2d getShallowRotation() {
        return getRotation().minus(m_headingOffset);
    }

    public void resetShallowHeading(Rotation2d r) {
        m_headingOffset = r;
    }

    public void resetShallowHeading() {
        if(CougarUtil.getAlliance() == Alliance.Red)
            resetShallowHeading(getRotation().plus(Rotation2d.k180deg));
        else
            resetShallowHeading(getRotation());
    }

    private SwerveRequest.ApplyRobotSpeeds req = new SwerveRequest.ApplyRobotSpeeds();

    private boolean m_rotDriftCorrect = true;

    private ChassisSpeeds rotationalDriftCorrection(ChassisSpeeds speeds) {
        ChassisSpeeds corrected = m_headingCorrector.update(speeds, m_state.Speeds, 
            super.getPigeon2().getRotation2d(), super.getPigeon2().getAngularVelocityZWorld().getValue().in(DegreesPerSecond));
        if (m_rotDriftCorrect && !DriverStation.isAutonomousEnabled())
        {
          return corrected;
        }
    
        return speeds;
    }

    private static final double[] kEmptyDoubleArr = {};
    public void drive(ChassisSpeeds s) {
        drive(s, null);
    }

    public void drive(ChassisSpeeds s, DriveFeedforwards ff) {
        req.Speeds = rotationalDriftCorrection(s);
        req.DesaturateWheelSpeeds = true;
        req.DriveRequestType = DriveRequestType.Velocity;
        req.SteerRequestType = SteerRequestType.Position;
        req.CenterOfRotation = Translation2d.kZero;
        if(ff != null) {
            req.WheelForceFeedforwardsX = ff.robotRelativeForcesXNewtons();
            req.WheelForceFeedforwardsY = ff.robotRelativeForcesYNewtons();
        } else {
            req.WheelForceFeedforwardsX = kEmptyDoubleArr;
            req.WheelForceFeedforwardsY = kEmptyDoubleArr;
        }
        super.setControl(req);
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }
}