package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{
    private final Intake m_intake;
    private final double m_targetRPM;

    public IntakeCommand(Intake m_intake, double m_targetRPM){
        this.m_intake = m_intake;
        this.m_targetRPM = m_targetRPM;

        addRequirements(m_intake);
    }

    @Override 
    public void initialize(){
        m_intake.setIntakeRPM(m_targetRPM);
    }

    @Override 
    public void execute(){
        m_intake.setIntakeRPM(m_targetRPM);
    }

    @Override 
    public boolean isFinished(){
        return false;
    }
    
}
