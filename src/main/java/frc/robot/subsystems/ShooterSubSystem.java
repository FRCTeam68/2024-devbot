package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubSystem extends SubsystemBase {

    public enum State{
        IDLE,
        SPINUP,
        SHOOT
    }

    public enum Mode{
        VOLTAGE_OUT,
        VOLTAGE_FOC,
        CURRENTTORQUE_FOC
    }

    private State m_presentState;
    private Mode m_presentMode;
    private double m_setPoint_Left_Speed;
    private double m_setPoint_Right_Speed;
    private double m_spinUp_Speed;
    private double m_rightOffset_Speed;
    private TalonFX m_shooterLeftMotor;
    private TalonFX m_shooterRightMotor;
    private VoltageOut m_voltageOut;
    private VelocityVoltage m_voltageVelocity;
    private VelocityTorqueCurrentFOC m_torqueVelocity;
    private NeutralOut m_neutral;
    private double m_setPoint_Left_Voltage;
    private double m_setPoint_Right_Voltage;
    private double m_rightOffset_Voltage;

    public ShooterSubSystem(){
        m_presentState = State.IDLE;
        m_presentMode = Mode.VOLTAGE_OUT;
        m_setPoint_Left_Speed = 0;
        m_setPoint_Right_Speed = 0;
        m_spinUp_Speed = Constants.SHOOTER.SPEED;
        m_rightOffset_Speed = Constants.SHOOTER.RIGHT_OFFSET;

        shooterMotorsInit();
    }

    private void shooterMotorsInit(){
        m_shooterLeftMotor = new TalonFX(Constants.SHOOTER.LEFT_CANID);
        m_shooterRightMotor = new TalonFX(Constants.SHOOTER.RIGHT_CANID);

        m_shooterLeftMotor.setInverted(true);
        m_shooterRightMotor.setInverted(false);

        m_voltageOut = new VoltageOut(0);

          /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
        m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, 
                                 false, false, false);
          /* Start at velocity 0, no feed forward, use slot 1 */
        m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, 
                                         false, false, false);
        /* Keep a neutral out so we can disable the motor */
        m_neutral = new NeutralOut();
  
        System.out.println("shooter subsystem created");
        

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        //0.11, 0.5, 0.0001, 0.12.  , .21 got there faster,  1 was too high (oscilated around zero when off)
        // still only getting to 20 instead of 40 that was being commanded
        configs.Slot0.kP = .2; // An error of 1 rotation per second results in 2V output  
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    
        // Peak output of 40 amps
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_shooterLeftMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to left shooter motor, error code: " + status.toString());
        }

        /* Retry config apply up to 5 times, report if failure */
        for (int i = 0; i < 5; ++i) {
          status = m_shooterRightMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to right shooter motor, error code: " + status.toString());
        }
    }

    public void setSpinUpSpeed(double desiredSpeed){
        m_spinUp_Speed=desiredSpeed;
    }

    public void setRightOffsetSpeed(double desiredSpeed){
        m_rightOffset_Speed=desiredSpeed;
    }

    public void setSpeedVout(double desiredVoltage){
        
        if(m_presentMode == Mode.VOLTAGE_OUT){
            if (Math.abs(desiredVoltage) <= .1) { // Joystick deadzone
                m_setPoint_Left_Voltage = 0;
                m_setPoint_Right_Voltage = 0;
            }
            else {
                m_setPoint_Left_Speed = desiredVoltage;
                m_setPoint_Right_Speed = -desiredVoltage + m_rightOffset_Voltage;
            }
            System.out.println("shooter setSpeedVout, L: " + m_setPoint_Left_Voltage 
                                                + " , R: " + m_setPoint_Right_Voltage);
            m_shooterLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
            m_shooterRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
        }
    }


    public void setSpeed(double desiredRotationsPerSecond){

        System.out.println("set desired speed: " + desiredRotationsPerSecond);

        if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
            desiredRotationsPerSecond = 0;
            m_setPoint_Left_Speed = 0;
            m_setPoint_Right_Speed = 0;
            m_shooterLeftMotor.setControl(m_neutral);
            m_shooterRightMotor.setControl(m_neutral);
        }
        else
            m_setPoint_Left_Speed = desiredRotationsPerSecond;
            m_setPoint_Right_Speed = -desiredRotationsPerSecond + m_rightOffset_Speed;

            System.out.println("shooter present mode: " + m_presentMode.toString());
            switch(m_presentMode){
                case VOLTAGE_OUT:
                    System.out.println("voltage out mode - use setSpeedVout instead");
                    break;
                default:
                case VOLTAGE_FOC:
                    /* Use voltage velocity */
                    //m_intakeMotor.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
                    System.out.println("call motor voltage foc");
                    m_shooterLeftMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Left_Speed));
                    m_shooterRightMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Right_Speed));
                    break;
        
                case CURRENTTORQUE_FOC:
                    double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
                    /* Use torque velocity */
                    //m_intakeMotor.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
                    System.out.println("call motor torqueVelocity");
                    m_shooterLeftMotor.setControl(m_torqueVelocity.withVelocity(m_setPoint_Left_Speed).withFeedForward(friction_torque));
                    m_shooterRightMotor.setControl(m_torqueVelocity.withVelocity(m_setPoint_Right_Speed).withFeedForward(friction_torque));
                    break;
            }
    }

    public double getSpinUpSpeed(){
        return this.m_spinUp_Speed;
    }

    public double getRightOffsetSpeed(){
        return this.m_rightOffset_Speed;
    }

    public double getLeftSpeed(){
        return this.m_setPoint_Left_Speed;
    }

    public double getRightSpeed(){
        return this.m_setPoint_Right_Speed;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");
        builder.setActuator(true);
        //builder.setSafeState(() -> setState(State.BREAK));
        //builder.addDoubleProperty("setpoint speed", null,this::setSpeed);
        builder.addDoubleProperty("left speed", this::getLeftSpeed,null);
        builder.addDoubleProperty("right speed", this::getRightSpeed,null);
        builder.addDoubleProperty("spinup speed", this::getSpinUpSpeed,this::setSpinUpSpeed);
        builder.addDoubleProperty("right offset speed", this::getRightOffsetSpeed,this::setRightOffsetSpeed);
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Mode", () -> m_presentMode.toString(),null);
    }


    public void setState(State wantedState) {
		m_presentState = wantedState;

        double desiredSpeed = 0;

        System.out.println("set shooter state: " + wantedState.toString());

        switch(wantedState){

            case SPINUP:   // L1
                desiredSpeed = m_spinUp_Speed;
                break;
            case SHOOT:     // L2
                // TBD
                // check if note present (top beam break is true)
                // check if shooter wheels are atspeed,
                // finally spin TopRoller at shoot speed, to feed note into shooter
                // wait 2 seconds, set speed back to 0
                break;
            default:
            case IDLE:
                //nothing, stay with zero speed
                break;
        }

        this.setSpeed(desiredSpeed);
	}

    public State getState(){
        return this.m_presentState;
    }

}