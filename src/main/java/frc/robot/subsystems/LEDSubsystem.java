package frc.robot.subsystems;


import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led = new AddressableLED(LED.PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.BUFFERSIZE);

  private boolean shooter = false;
  private boolean redBall = false;
  private boolean blueBall = false;

  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    System.out.println("getlength:"+ m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {

    if (DriverStation.isEnabled()) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.equals(Alliance.Red)) {

        redBall = true;
      }
      else if (ally.equals(Alliance.Blue)) { // should only have pipelines 0 & 1

        blueBall = true;
      }
      setLEDs();
    }
  }

  @Override
  public void simulationPeriodic() {}

  public void setShooter(boolean ready){
    
    shooter=ready;
  }

  private void setLEDs() {
    // setBallLEDs();
    setShooterLEDs();
    m_led.setData(m_ledBuffer);
  }

  private void setBallLEDs() {
    if (redBall && blueBall) {
      setFrontHalf();
    }
    else if (redBall || blueBall) {
      if (redBall) {
        setFrontAll(Color.kRed);
      }
      if (blueBall) {
        setFrontAll(Color.kBlue);
      }
    }
    else {
      setFrontAll(Color.kBlack); // Off
    }
  }

  private void setShooterLEDs() {
    // System.out.println("shooterLEDs");
    if (shooter) {
      // setBackAll(Color.kGreen);
      m_ledBuffer.setLED(1, Color.kGreen);
    }
    else {
      // setBackAll(Color.kBlack); // Off
      m_ledBuffer.setLED(1, Color.kBlack);
    }
  }

  private void setFrontAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      m_ledBuffer.setLED(i, color);
    }
  }

  public void setFrontHalf() {
    for (int i = 0; i < m_ledBuffer.getLength() / 2; i++) {
      if (i < m_ledBuffer.getLength() / 2) {
        m_ledBuffer.setLED(i, Color.kBlue);
      }
      else {
        m_ledBuffer.setLED(i, Color.kRed);
      }
    }
  }

  public void setBackAll(Color color) {
    System.out.println("setBackAll:"+ color);
    for (var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
      System.out.println("setBackAll:"+ i+ ", color: " + color);
      m_ledBuffer.setLED(i, color);
    }
  }
}