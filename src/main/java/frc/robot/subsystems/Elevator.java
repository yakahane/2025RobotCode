// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.ExpandedSubsystem;

@Logged
public class Elevator extends ExpandedSubsystem {
  /** Creates a new Elevator. */
  private TalonFX elevatorMainMotor;

  private TalonFX elevatorFollowerMotor;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private DigitalInput buttonSwitch = new DigitalInput(ElevatorConstants.buttonSwitchID);
  private boolean isZeroed = false;
  private Alert elevatorAlert;
  private boolean lastButtonState = false;
  private Debouncer buttonDebouncer = new Debouncer(.28);
  private Debouncer elevatorDebouncer = new Debouncer(.6);
  private Debouncer zeroedDebouncer = new Debouncer(2.5);

  private double currentPosition;
  private double positionTolerance = Units.inchesToMeters(.2);

  public Elevator() {
    elevatorMainMotor = new TalonFX(ElevatorConstants.elevatorMainMotorID);
    elevatorFollowerMotor = new TalonFX(ElevatorConstants.elevatorFollowerMotorID);
    // follower = new Follower(ElevatorConstants.elevatorMainMotorID, false);
    elevatorMainMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);
    elevatorFollowerMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);

    // elevatorFollowerMotor.setControl(follower);

    elevatorAlert = new Alert("Elevator is not Zeroed!", AlertType.kWarning);
    // elevatorMainMotor.setPosition(0.0);
    // elevatorFollowerMotor.setPosition(0.0);
  }

  public boolean buttonPressed() {
    return !buttonSwitch.get();
  }

  public Command homeElevator() {
    return downSpeed(0.1)
        .until(() -> buttonDebouncer.calculate(buttonPressed()))
        .unless(() -> buttonDebouncer.calculate(buttonPressed()))
        .finallyDo(this::stopElevator)
        .withName("Home elevator");
  }

  public void stopElevator() {
    elevatorMainMotor.set(0);
    elevatorFollowerMotor.set(0);
  }

  public boolean elevatorIsDown() {
    return Units.metersToInches(elevatorMainMotor.getPosition().getValueAsDouble()) < 6;
  }

  public Command upSpeed(double speed) {
    return run(
        () -> {
          elevatorMainMotor.set(speed);
          elevatorFollowerMotor.set(speed);
        });
  }

  public Command downSpeed(double speed) {
    return run(() -> {
          elevatorMainMotor.set(-speed);
          elevatorFollowerMotor.set(-speed);
        })
        .until(() -> buttonDebouncer.calculate(buttonPressed()))
        .unless(() -> buttonDebouncer.calculate(buttonPressed()));
  }

  public Command moveToPosition(double height) {
    double h = height + Units.inchesToMeters(.2);
    return run(() -> {
          elevatorMainMotor.setControl(motionMagicRequest.withPosition(h));
          elevatorFollowerMotor.setControl(motionMagicRequest.withPosition(h));
        })
        .until(() -> (atSetPoint(height)))
        // .onlyIf(() -> isZeroed)
        .withName("move to position");
    // .finallyDo(this::holdPosition);
  }

  public Command downPosition() {
    return run(() -> {
          elevatorMainMotor.setControl(
              motionMagicRequest.withPosition(ElevatorConstants.downHeight));
          elevatorFollowerMotor.setControl(
              motionMagicRequest.withPosition(ElevatorConstants.downHeight));
        })
        .until(() -> (atSetPoint(ElevatorConstants.downHeight)))
        .andThen(downSpeed(.01).until(() -> buttonDebouncer.calculate(buttonPressed())))
        .withName("Down position");
  }

  public boolean atSetPoint(double targetHeight) {
    return elevatorDebouncer.calculate(
        Math.abs(targetHeight - elevatorMainMotor.getPosition().getValueAsDouble())
            < positionTolerance);
  }

  public Command holdPosition() {
    return startRun(
            () -> {
              elevatorMainMotor.setControl(
                  motionMagicRequest.withPosition(
                      elevatorMainMotor.getPosition().getValueAsDouble()));
              elevatorFollowerMotor.setControl(
                  motionMagicRequest.withPosition(
                      elevatorFollowerMotor.getPosition().getValueAsDouble()));
            },
            () -> {
              elevatorMainMotor.setControl(motionMagicRequest);
              elevatorFollowerMotor.setControl(motionMagicRequest);
            })
        .withName("Elevator Hold");
  }

  private final SysIdRoutine elevatorSysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(1.0).per(Second), // Use default ramp rate (1 V/s)
              Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdElevator_State", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> {
                elevatorMainMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
                elevatorFollowerMotor.setControl(voltageRequest.withOutput(volts.in(Volts)));
              },
              log -> {
                log.motor("Main")
                    .angularVelocity(
                        Rotations.per(Minute)
                            .of(elevatorMainMotor.getVelocity().getValueAsDouble()))
                    .angularPosition(
                        Rotations.of(elevatorMainMotor.getVelocity().getValueAsDouble()));
                log.motor("Follower")
                    .angularVelocity(
                        Rotations.per(Minute)
                            .of(elevatorFollowerMotor.getVelocity().getValueAsDouble()))
                    .angularPosition(
                        Rotations.of(elevatorFollowerMotor.getVelocity().getValueAsDouble()));
              },
              this));

  public Command sysIdQuasistaticElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicElevator(SysIdRoutine.Direction direction) {
    return elevatorSysIdRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Elevator Main Position",
        Units.metersToInches(elevatorMainMotor.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Elevator Follower Position",
        Units.metersToInches(elevatorFollowerMotor.getPosition().getValueAsDouble()));
    SmartDashboard.putBoolean("Button Pressed", buttonPressed());

    boolean currentButtonState = buttonPressed();

    if (currentButtonState && !lastButtonState) {
      elevatorMainMotor.setPosition(0, 0);
      elevatorFollowerMotor.setPosition(0, 0);

      isZeroed = true;
    }

    lastButtonState = currentButtonState;

    if (isZeroed
        && zeroedDebouncer.calculate(
            Units.metersToInches(elevatorMainMotor.getPosition().getValueAsDouble()) < 1)
        && !buttonPressed()) {
      isZeroed = false;
    }

    elevatorAlert.set(!isZeroed);
  }
}
