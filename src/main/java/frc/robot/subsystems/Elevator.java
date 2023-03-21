// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.core238.PoseHelper;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ElevatorParameters;
public class Elevator extends SubsystemBase {
  protected final static ElevatorFeedforward FF = new ElevatorFeedforward(ElevatorParameters.ks, ElevatorParameters.kg, ElevatorParameters.kv, ElevatorParameters.ka);
  protected final static CANSparkMax elevatorLeader = RobotMap.ElevatorParameters.elevatorLeader;
  protected final static CANSparkMax elevatorFollower = RobotMap.ElevatorParameters.elevatorFollower;
  protected Pose3d elevatorPose = new Pose3d();
  protected GenericEntry poseEntry;

  protected DoubleArrayLogEntry logPose;
  protected DoubleLogEntry logEncoder;
  protected DoubleLogEntry logDesiredV;
  protected DoubleLogEntry logActualV;
  protected DoubleLogEntry logDesiredEncoder;
  protected DoubleLogEntry logFF;
  protected StringLogEntry logCommand;
  protected BooleanLogEntry logLowerLimit;
  protected BooleanLogEntry logUpperLimit;

  private final DCMotor m_simMotors = DCMotor.getNEO(2);
  private final ElevatorSim m_sim = new ElevatorSim(m_simMotors, 4, Units.lbsToKilograms(10), Units.inchesToMeters(.75), 0, Units.inchesToMeters(36), true, VecBuilder.fill(0));
  private final PWMSparkMax m_simMotor = new PWMSparkMax(0);
  private final PWMSim m_motorSim = new PWMSim(m_simMotor);
  

  private static boolean debug = false;
  
  /** Creates a new Elevator. */
  public Elevator() {
    initSparkMax();
    SmartDashboard.putBoolean("ElevatorDebugging", debug);
    logPose = new DoubleArrayLogEntry(DataLogManager.getLog(), "Elevator:Pose");
    logEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Elevator:Position");
    logDesiredV = new DoubleLogEntry(DataLogManager.getLog(), "Elevator:Desired V");
    logActualV = new DoubleLogEntry(DataLogManager.getLog(), "Elevator:Actual V");
    logDesiredEncoder = new DoubleLogEntry(DataLogManager.getLog(), "Elevator:Desired Position");
    logFF = new DoubleLogEntry(DataLogManager.getLog(), "Elevator:FF");
    logLowerLimit = new BooleanLogEntry(DataLogManager.getLog(), "Elevator:LowerLimit");
    logUpperLimit = new BooleanLogEntry(DataLogManager.getLog(), "Elevator:UpperLimit");
    logCommand = new StringLogEntry(DataLogManager.getLog(), "Elevator:Command");
  }
  
  public void initSparkMax() {
    elevatorFollower.follow(elevatorLeader);
    elevatorLeader.setSmartCurrentLimit(RobotMap.ElevatorParameters.sparkCurrentLimit);
    elevatorFollower.setSmartCurrentLimit(RobotMap.ElevatorParameters.sparkCurrentLimit);
    //elevatorLeader.setSoftLimit(SoftLimitDirection.kForward, (float)RobotMap.ElevatorParameters.softLimitForward);
    //elevatorLeader.setSoftLimit(SoftLimitDirection.kReverse, (float)RobotMap.ElevatorParameters.softLimitBackward);
    elevatorLeader.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
    elevatorLeader.getPIDController().setP(ElevatorParameters.kp);
    elevatorLeader.getPIDController().setI(ElevatorParameters.ki);
    elevatorLeader.getPIDController().setD(ElevatorParameters.kd);
    elevatorLeader.getPIDController().setOutputRange(-.2, 1);
    elevatorLeader.getEncoder().setPosition(0);
    elevatorLeader.getEncoder().setAverageDepth(2);
    elevatorLeader.getEncoder().setMeasurementPeriod(16);
  }

  public void moveByPercentOutput(double percent) {
    if(Robot.isReal()){
      elevatorLeader.set(percent);
    } else {
      m_simMotor.set(percent);
    }
  }

  public double getEncoderPosition() {
    if(Robot.isReal()) {
      return elevatorLeader.getEncoder().getPosition();
    } else
    {
      return RobotMap.ElevatorParameters.inchesToTicks(Units.metersToInches(m_sim.getPositionMeters()));
    }
  }

  public double getPositionMeters() {
    //1.5*Pi inches per pulley rotation divided by 4:1 gear ratio
    return Units.inchesToMeters(1.5*Math.PI*getEncoderPosition()/(4));
  }

  public Pose3d getPose() {
    return new Pose3d(getPositionMeters()*Math.sin(Math.toRadians(35)), 0, getPositionMeters()*Math.cos(Math.toRadians(35)), new Rotation3d());
  }
  
  @Override
  public void periodic() {

    debug = SmartDashboard.getBoolean("ElevatorDebugging", debug);
    
    // This method will be called once per scheduler run
    if(elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed()) {
    }
    
    logPose.append(PoseHelper.PoseToArray(getPose()));
    if(SmartDashboard.getBoolean("ElevatorDebugging", debug))
    {
      SmartDashboard.putNumber("Elevator Encoder", getEncoderPosition());
      SmartDashboard.putBoolean("Elevator Lower Limit", getLowerLimit());
      SmartDashboard.putBoolean("Elevator Upper Limit", getUpperLimit());
      NetworkTableInstance.getDefault().flush();
    } else {
      logLowerLimit.append(getLowerLimit());
      logUpperLimit.append(getUpperLimit());
      logEncoder.append(getEncoderPosition());
    }
  }
  
  public void putCommandString(Command command) {
    if(SmartDashboard.getBoolean("ElevatorDebugging", debug)){
      SmartDashboard.putString("Elevator Command", command.getName());
    } else {
      logCommand.append(command.getName());
    }
  }

  public void PIDdrive(TrapezoidProfile.State currentState, TrapezoidProfile.State nextState) {
    double feed = FF.calculate(currentState.velocity, (nextState.velocity - currentState.velocity)/.02);

    if(SmartDashboard.getBoolean("ElevatorDebugging", debug))
    {
      SmartDashboard.putNumber("Elevator Desired V", currentState.velocity);
      SmartDashboard.putNumber("Elevator Desired Pos", currentState.position);
      SmartDashboard.putNumber("Elevator Actual V", getVelocity());
      SmartDashboard.putNumber("Elevator Actual Pos", getEncoderPosition());
      SmartDashboard.putNumber("Elevator FF", feed);
    } else {
      logDesiredV.append(currentState.velocity);
      logDesiredEncoder.append(currentState.position);
      logActualV.append(getVelocity());
      logFF.append(feed);
    }

    if(Robot.isReal()){
    elevatorLeader.getPIDController().setReference(currentState.position, ControlType.kPosition, 0, feed);
    }
    else {
      feed = feed + 1*(currentState.position - getEncoderPosition());
      m_simMotor.setVoltage(feed);
    }
  }

  public double getVelocity() {
    if(Robot.isReal())
    {
    return elevatorLeader.getEncoder().getVelocity()/60;
    }
    else {
      return RobotMap.ElevatorParameters.inchesToTicks(Units.metersToInches(m_sim.getVelocityMetersPerSecond()));
    }
  }

  public boolean getLowerLimit() {
    return elevatorLeader.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public boolean getUpperLimit() {
    return elevatorLeader.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_sim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_sim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_sim.getCurrentDrawAmps()));
  }
}
