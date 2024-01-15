package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubSystem extends SubsystemBase {

    public enum State{
        IDLE,
        INTAKING_CONE,
        INTAKING_CUBE,
        IDLE_CUBE,
        PLACING,
        SHOOT
    }

    public enum Mode{
        VOLTAGE_FOC,
        CURRENTTORQUE_FOC
    }

    private State m_presentState;
    private Mode m_presentMode;
    private double m_setSpeed;
    private TalonFX m_intakeMotor;
    private VelocityVoltage m_voltageVelocity;
    private VelocityTorqueCurrentFOC m_torqueVelocity;
    private NeutralOut m_brake;

    public IntakeSubSystem(){
        m_presentState = State.IDLE;
        m_presentMode = Mode.VOLTAGE_FOC;
        m_setSpeed = 0;
        m_intakeMotor = new TalonFX(14, "MANIPbus");

          /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
        m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, 
                                 false, false, false);
          /* Start at velocity 0, no feed forward, use slot 1 */
        m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, 
                                         false, false, false);
        /* Keep a neutral out so we can disable the motor */
        m_brake = new NeutralOut();
        

        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 8;
        configs.Voltage.PeakReverseVoltage = -8;
        
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    
        // Peak output of 40 amps
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    
        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_intakeMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to intake, error code: " + status.toString());
        }
    }

    public Command setIntakeSpeedCommand(double desiredRotationsPerSecond){
        Command toRun;

        if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
            desiredRotationsPerSecond = 0;
            m_setSpeed = 0;
            /* Disable the motor instead */
            //m_intakeMotor.setControl(m_brake);
            toRun = Commands.runOnce(() -> m_intakeMotor.setControl(m_brake));
        }
        else
            m_setSpeed = desiredRotationsPerSecond;

            switch(m_presentMode){
                default:
                case VOLTAGE_FOC:
                    /* Use voltage velocity */
                    //m_intakeMotor.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
                    toRun = Commands.runOnce(() -> 
                            m_intakeMotor.setControl(m_voltageVelocity.withVelocity(m_setSpeed)));
                    break;
        
                case CURRENTTORQUE_FOC:
                    double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
                    /* Use torque velocity */
                    //m_intakeMotor.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
                    toRun = Commands.runOnce(() -> 
                            m_intakeMotor.setControl(m_torqueVelocity.withVelocity(m_setSpeed).withFeedForward(friction_torque)));
                    break;
            }
        return toRun;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // Publish the intake state to telemetry.
        builder.addDoubleProperty("Speed", () -> m_setSpeed,null);
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Mode", () -> m_presentMode.toString(),null);
    }

    public Command setStateCommand(State wantedState) {
		m_presentState = wantedState;

        double desiredSpeed = 0;

        switch(wantedState){

            case INTAKING_CONE:   // L1ButtonPressed
                desiredSpeed = -50; //-1);
                break;
            case INTAKING_CUBE:   // L2ButtonPressed
                desiredSpeed = -15; //-.3);
                break;
            case PLACING:         // R2ButtonPressed
                desiredSpeed = 10; //.15);
                break;
            // case IDLE_CUBE:
            //     desiredSpeed = -5; //-.1);
            //     break;
            // case SHOOT:
            //     desiredSpeed = 50; //1);
            //     break;
            default:
            case IDLE:
                desiredSpeed = 0;
                break;
        }
        return setIntakeSpeedCommand(desiredSpeed);
	}

    public State getState(){
        return this.m_presentState;
    }

}