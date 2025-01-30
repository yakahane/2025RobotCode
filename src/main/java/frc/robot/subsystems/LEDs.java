// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {

  public AddressableLED addressableLED = new AddressableLED(LEDConstants.ledPort);
    public AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.bufferLength);
  /** Creates a new LED. */

  public LED() {

    addressableLED.setLength(buffer.getLength());
    addressableLED.setData(buffer);
    addressableLED.start();
    
  }

  public AddressableLEDBuffer  getAddressableLEDBuffer = new AddressableLEDBuffer(120);
  
  public LEDPattern red = LEDPattern.solid(Color.kRed);


  
  
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
