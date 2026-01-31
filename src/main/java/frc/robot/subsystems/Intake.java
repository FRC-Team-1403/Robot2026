// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import edu.wpi.first.math.controller.ProfiledPIDController;


// public class Intake extends SubsystemBase{
//     private final SparkMax m_motor1;
//     private final RelativeEncoder m_encoder;
//     private SimpleMotorFeedForward m_feedforward;

//     public Intake (){
//         m_motor1 = new SparkMax(0, MotorType.kBrushless);
        
//         SparkMaxConfig config1 = new SparkMaxConfig();
//         config1.idleMode(IdleMode.kCoast);
//         config1.smartCurrentLimit(40);

//         m_motor1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

//         m_encoder = m_motor1.getEncoder();

//         m_feedForward = new SimpleMotorFeedforward(
//             Constants.Intake.kS,
//             Constants.Intake.kV,
//             Constants.Intake.kA
//         );
//         m_profiled = new ProfiledPIDController(
//       Constants.Intake.kP,
//       Constants.Intake.kI,
//       Constants.Intake.kD,
//       new TrapezoidProfile.Constraints(
//         Constants.Shooter.maxVelocityRPM,
//         Constants.Shooter.maxAccelerationRPMPerSec
//       )
//     );
    
//     m_profiled.setTolerance(Constants.Shooter.rpmTolerance);
        


//     }

// }
