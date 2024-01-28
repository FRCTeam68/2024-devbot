package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AngleSubSystem extends SubsystemBase {

    public enum State{
        SPEAKER,
        AMP,
        TRAP,
        FEED
    }

    public enum Mode{
        MMV,
        MMV_FOC,
        VOUT
    }

    private State m_presentState;
    private Mode m_presentMode;

    private double m_setPoint_Position;
    private double m_speaker_position;
    private double m_amp_position;
    private double m_trap_position;
    private double m_feed_position;

    private TalonFX m_angleMotor;
    private MotionMagicVoltage m_angleMotorMMV;
    private NeutralOut m_brake;

    public AngleSubSystem(){
        m_presentState = State.SPEAKER;
        m_presentMode = Mode.MMV;
        m_speaker_position = Constants.ANGLE.SPEAKER;
        m_amp_position = Constants.ANGLE.AMP;
        m_trap_position = Constants.ANGLE.TRAP;
        m_feed_position = Constants.ANGLE.FEED;

        angleMotorInit();
    }

    private void angleMotorInit(){
        m_angleMotor = new TalonFX(Constants.ANGLE.CANID);
        m_angleMotorMMV = new MotionMagicVoltage(Constants.ANGLE.SPEAKER);  

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   

		m_angleMotor.setInverted(true);
        
        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_angleMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate motor. Error: " + status.toString());
        }

        m_angleMotorMMV.OverrideBrakeDurNeutral = true;
        m_angleMotor.setVoltage(0);
        // m_angleMotor.setSafetyEnabled(false);

        zeroAngleSensor(); 
    }

    public void zeroAngleSensor(){
        // m_rotateMotor.setRotorPosition(0);  no setRotorPosition anymore.  does setPosition do the same thing???
        m_angleMotor.setPosition(0);
        //move arm to speaker '0' position
        //  No need to zero.   absolute CAN coder position will be used.
        //  so if it starts off zero, it will go to zero upon going to Nuetral state 
        //NOPE, magnet offset did not work.   go back to set rotor to zero.
    }

    public void setPosition(double desiredPosition){

        System.out.println("set angle desired position: " + desiredPosition);

        m_setPoint_Position = desiredPosition;

        System.out.println("angle present mode: " + m_presentMode.toString());
        switch(m_presentMode){
            default:
            case MMV:
                System.out.println("call motor MMV ");
                m_angleMotor.setControl(m_angleMotorMMV.withPosition(m_setPoint_Position));
                break;
    
            case MMV_FOC:
                //TBD
                break;
        }
    }

    public void setSpeakerPosition(double desiredPosition) {
        m_speaker_position = desiredPosition;
	}
    public void setAmpPosition(double desiredPosition){
        m_amp_position = desiredPosition;
    }
    public void setTrapPosition(double desiredPosition) {
        m_trap_position = desiredPosition;
	}
    public void setFeedPosition(double desiredPosition){
        m_feed_position = desiredPosition;
    }


    public double getPosition(){
        return this.m_setPoint_Position;
    }

    public double getSpeakerPosition(){
        return this.m_speaker_position;
    }
    public double getAmpPosition(){
        return this.m_amp_position;
    }
    public double getTrapPosition(){
        return this.m_trap_position;
    }
    public double getFeedPosition(){
        return this.m_feed_position;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Angle");
        builder.setActuator(true);
        // builder.setSafeState(() -> setState(State.BREAK));
        builder.addDoubleProperty("setpoint position", this::getPosition,this::setPosition);
        builder.addDoubleProperty("speaker position", this::getSpeakerPosition,this::setSpeakerPosition);
        builder.addDoubleProperty("amp position", this::getAmpPosition,this::setAmpPosition);
        builder.addDoubleProperty("trap position", this::getTrapPosition,this::setTrapPosition);
        builder.addDoubleProperty("feed position", this::getFeedPosition,this::setFeedPosition);
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Mode", () -> m_presentMode.toString(),null);
    }

    public void setState(State wantedState) {
		m_presentState = wantedState;

        double desiredPosition = 0;

        System.out.println("set state command: " + wantedState.toString());

        switch(wantedState){
            default:
            case SPEAKER:
                desiredPosition = m_speaker_position;
                break;
            case AMP:
                desiredPosition = m_amp_position;;
                break;
            case TRAP:
                desiredPosition = m_trap_position;
                break;
            case FEED:
                desiredPosition = m_feed_position;
                break;
        }

        setPosition(desiredPosition);    
	}

    public State getState(){
        return this.m_presentState;
    }

}