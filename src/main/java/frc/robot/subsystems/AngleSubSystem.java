package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
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
        INTAKE,
        FEEDSTATION,
        SPEAKER_PODIUM
    }

    public enum Mode{
        MMV,
        MMV_FOC,
        VOUT
    }

    private State m_presentState;
    private Mode m_presentMode;

    private double m_setPoint_Position;
    private double m_setPoint_Adjust;
    private double m_speaker_position;
    private double m_amp_position;
    private double m_trap_position;
    private double m_intake_position;

    private TalonFX m_angleLeftMotor;
    private TalonFX m_angleRightMotor;
    private MotionMagicVoltage m_angleMotorMMV;
    private NeutralOut m_brake;
    private Timer m_bumpTimer;
    private double m_bumpCount;

    public AngleSubSystem(){
        m_presentState = State.SPEAKER;
        m_presentMode = Mode.MMV;
        m_speaker_position = Constants.ANGLE.SPEAKER;
        m_amp_position = Constants.ANGLE.AMP;
        m_trap_position = Constants.ANGLE.TRAP;
        m_intake_position = Constants.ANGLE.INTAKE;
        m_setPoint_Position = Constants.ANGLE.SPEAKER;
        m_setPoint_Adjust = 0;
        m_bumpTimer = new Timer();
        m_bumpTimer.start();
        m_bumpCount = 0;

        angleMotorInit();
    }

    private void angleMotorInit(){
        m_angleLeftMotor = new TalonFX(Constants.ANGLE.LEFT_CANID, "rio");
        m_angleRightMotor = new TalonFX(Constants.ANGLE.RIGHT_CANID, "rio");
        m_angleRightMotor.setControl(new Follower(Constants.ANGLE.LEFT_CANID, true));

        m_angleMotorMMV = new MotionMagicVoltage(Constants.ANGLE.SPEAKER);  

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        /* Configure current limits */
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80; //106; // 5 rotations per second cruise
        cfg.MotionMagic.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
        cfg.MotionMagic.MotionMagicJerk = 700;   

        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		// m_angleLeftMotor.setInverted(true);
        
        cfg.Slot0.kP = 55.0F;
        cfg.Slot0.kI = 0.0F;
        cfg.Slot0.kD = 0.0F;
        cfg.Slot0.kV = 0.0F;
        cfg.Slot0.kS = 0.25F; // Approximately 0.25V to get the mechanism moving

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 30.0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = m_angleLeftMotor.getConfigurator().apply(cfg);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure rotate motor. Error: " + status.toString());
        }

        m_angleMotorMMV.OverrideBrakeDurNeutral = true;
        m_angleLeftMotor.setVoltage(0);
        // m_angleMotor.setSafetyEnabled(false);

        zeroAngleSensor(); 
    }

    public void zeroAngleSensor(){
        // m_rotateMotor.setRotorPosition(0);  no setRotorPosition anymore.  does setPosition do the same thing???
        m_angleLeftMotor.setPosition(0);
        //move arm to speaker '0' position
        //  No need to zero.   absolute CAN coder position will be used.
        //  so if it starts off zero, it will go to zero upon going to Nuetral state 
        //NOPE, magnet offset did not work.   go back to set rotor to zero.
    }

    public void setPositionJoy(double desiredAjustPosition){
        m_bumpCount = m_bumpCount + 1;
        if ((Math.abs(desiredAjustPosition)>0.5) && (m_bumpTimer.hasElapsed(1))) {
            m_bumpTimer.restart();
            
            m_setPoint_Adjust = m_setPoint_Adjust + desiredAjustPosition*Constants.ANGLE.BUMP_VALUE;
            System.out.println("desired: " + desiredAjustPosition
                               + "setpoint: " + m_setPoint_Position 
                               + ", plus: " + m_setPoint_Adjust
                               + ", count: " + m_bumpCount);
            setPosition(m_setPoint_Position + m_setPoint_Adjust);
        }

        // switch(m_presentMode){
        //     default:
        //     case MMV:
        //         System.out.println("angle setPositionJoy: " + desiredPosition);
        //         m_angleLeftMotor.setControl(m_angleMotorMMV.withPosition(desiredPosition));
        //         break;
        //     case MMV_FOC:
        //         //TBD
        //         break;
        // }
    }

    public void setPosition(double desiredPosition){

        System.out.println("set angle desired position: " + desiredPosition);

        m_setPoint_Position = desiredPosition;

        System.out.println("angle present mode: " + m_presentMode.toString());
        switch(m_presentMode){
            default:
            case MMV:
                System.out.println("call motor MMV ");
                m_angleLeftMotor.setControl(m_angleMotorMMV.withPosition(m_setPoint_Position));
                break;
    
            case MMV_FOC:
                //TBD
                break;
        }
    }

    public boolean atAngle(){
        double motorPosition = m_angleLeftMotor.getPosition().getValueAsDouble();
        System.out.println("at angle: " + motorPosition);
        return Math.abs(m_setPoint_Position-motorPosition) < 1.0;
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
    public void setIntakePosition(double desiredPosition){
        m_intake_position = desiredPosition;
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
    public double getIntakePosition(){
        return this.m_intake_position;
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
        builder.addDoubleProperty("intake position", this::getIntakePosition,this::setIntakePosition);
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Mode", () -> m_presentMode.toString(),null);
    }

    public void setState(State wantedState) {
		m_presentState = wantedState;

        double desiredPosition = 0;

        System.out.println("set angle state: " + wantedState.toString());

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
            case INTAKE:
                desiredPosition = m_intake_position;
                break;
        }
        m_setPoint_Adjust = 0;

        setPosition(desiredPosition);    
	}

    public State getState(){
        return this.m_presentState;
    }

}