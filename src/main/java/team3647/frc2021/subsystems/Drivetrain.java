package team3647.frc2021.subsystems;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.ControlType;

import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import team3647.lib.drivers.SparkMaxFactory.Configuration;
import team3647.lib.drivers.ClosedLoopFactory;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.SparkMaxUtil;
import team3647.lib.util.DriveSignal;
import team3647.lib.wpi.HALMethods;
import team3647.lib.wpi.Timer;




public class Drivetrain implements PeriodicSubsystem{
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave1;
    private CANSparkMax leftSlave2;
    private CANSparkMax rightMaster;
    private CANSparkMax rightSlave1;
    private CANSparkMax rightSlave2;
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;
    private final Configuration m_leftMasterConfig;
    private final Configuration m_rightMasterConfig;

    private final ClosedLoopConfig m_leftPIDConfig;
    private final ClosedLoopConfig m_rightPIDConfig;

    private CANPIDController m_leftVelocityPID;
    private CANPIDController m_rightVelocityPID;
    private final double kEncoderVelocityToMetersPerSecond;    


    private PeriodicIO periodicIO = new PeriodicIO();
    private boolean initialized = false;
    private double m_timeStamp;
    private double m_maxOutput;
    private ControlType controlType = ControlType.kDutyCycle;

    private final SimpleMotorFeedforward feedforward;

    public Drivetrain(Configuration leftMasterConfig, Configuration rightMasterConfig,
    Configuration leftSlave1Config, Configuration rightSlave1Config, 
    Configuration leftSlave2Config, Configuration rightSlave2Config,
     ClosedLoopConfig leftMasterPIDConfig, ClosedLoopConfig rightMasterPIDConfig,
    double kWheelDiameterMeters,double kS, double kV, double kA){

        m_leftMasterConfig = leftMasterConfig;
        m_rightMasterConfig = rightMasterConfig;

        m_leftPIDConfig = leftMasterPIDConfig;
        m_rightPIDConfig = rightMasterPIDConfig;

        leftMaster = SparkMaxFactory.createSparkMax(m_leftMasterConfig);
        rightMaster = SparkMaxFactory.createSparkMax(m_rightMasterConfig);
        

        leftSlave1 = SparkMaxFactory.createSparkMaxFollower(leftMaster, leftSlave1Config);
        rightSlave1 = SparkMaxFactory.createSparkMaxFollower(rightMaster, rightSlave1Config);
        leftSlave2 = SparkMaxFactory.createSparkMaxFollower(leftMaster, leftSlave2Config);
        rightSlave2 = SparkMaxFactory.createSparkMaxFollower(rightMaster, rightSlave2Config);

        leftEncoder=leftMaster.getEncoder();
        rightEncoder=rightMaster.getEncoder();

        m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster, leftEncoder,
        m_leftPIDConfig, 0);
        m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
        rightEncoder, m_rightPIDConfig, 0);

        kEncoderVelocityToMetersPerSecond =
        m_leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameterMeters * Math.PI / 60.0;
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        
    }
    
    public static class PeriodicIO {
        // inputs
        /** Meters per second */
        public double leftVelocity;
        /** Meters per second */
        public double rightVelocity;

        public boolean climbLimitSwitch;
        public double ypr[] = new double[] {0, 0, 0};

        /** Meters */
        public double leftPosition;
        /** Meters */
        public double rightPosition;
        /** Degrees -180 to 180 */
        public double heading;

        // outputs
        public double leftOutput;
        public double rightOutput;

        /** Meters Per second */
        public double prevLeftDesiredVelocity;
        /** Meters Per Second */
        public double prevRightDesiredVelocity;

        /** Volts */
        public double leftFeedForward;
        /** Volts */
        public double rightFeedForward;
    }
    
    @Override
    public synchronized void init() {
        setToBrake();
        resetEncoders();
        initialized=true;
    }
    
    public synchronized void resetEncoders() {
        try {
            SparkMaxUtil.checkError(leftEncoder.setPosition(0),
                    m_timeStamp + " Couldn't reset left encoder");
        } catch (NullPointerException e) {
            leftEncoder = leftMaster.getEncoder();
            m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster,
                    leftEncoder, m_leftPIDConfig, 0);
            HALMethods.sendDSError(e.toString());
        }

        try {
            SparkMaxUtil.checkError(rightEncoder.setPosition(0),
                    m_timeStamp + " Couldn't reset right encoder");
        } catch (NullPointerException e) {
            rightEncoder = rightMaster.getEncoder();
            m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                    rightEncoder, m_rightPIDConfig, 0);
            HALMethods.sendDSError(e.toString());
        }
        periodicIO = new PeriodicIO();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        try {
            periodicIO.leftPosition =
                    leftEncoder.getPosition() * m_leftPIDConfig.kEncoderTicksToUnits;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructLeftEncoder();
            periodicIO.leftPosition =
                    leftEncoder.getPosition() * m_leftPIDConfig.kEncoderTicksToUnits;
        }

        try {
            periodicIO.rightPosition =
                    rightEncoder.getPosition() * m_rightPIDConfig.kEncoderTicksToUnits;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructRightEncoder();
            periodicIO.rightPosition =
                    rightEncoder.getPosition() * m_rightPIDConfig.kEncoderTicksToUnits;
        }

        try {
            periodicIO.leftVelocity = leftEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructLeftEncoder();
            periodicIO.leftVelocity = leftEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        }

        try {
            periodicIO.rightVelocity =
                    rightEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructRightEncoder();
            periodicIO.rightVelocity =
                    rightEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        }
    }

    private void reconstructLeftEncoder() {
        leftEncoder = leftMaster.getEncoder();
        m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster, leftEncoder,
                m_leftPIDConfig, 0);
    }

    private void reconstructRightEncoder() {
        rightEncoder = leftMaster.getEncoder();
        m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                rightEncoder, m_rightPIDConfig, 0);
    }

    @Override
    public synchronized void writePeriodicOutputs(){
        try {
            if (controlType == ControlType.kDutyCycle) {
                leftMaster.set(periodicIO.leftOutput);
            } else {
                m_leftVelocityPID.setReference(periodicIO.leftOutput, controlType, 0,
                        periodicIO.leftFeedForward);
            }
        } catch (NullPointerException e) {
            m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster,
                    leftEncoder, m_leftPIDConfig, 0);
            m_leftVelocityPID.setReference(periodicIO.leftOutput, controlType, 0,
                    periodicIO.leftFeedForward);
            HALMethods.sendDSError(e.toString());
        }

        try {
            if (controlType == ControlType.kDutyCycle) {
                rightMaster.set(periodicIO.rightOutput);
            } else {
                m_rightVelocityPID.setReference(periodicIO.rightOutput, controlType, 0,
                        periodicIO.rightFeedForward);
            }
        } catch (NullPointerException e) {
            m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                    rightEncoder, m_rightPIDConfig, 0);
            m_rightVelocityPID.setReference(periodicIO.rightOutput, controlType, 0,
                    periodicIO.rightFeedForward);
            HALMethods.sendDSError(e.toString());
        }
    }

    @Override
    public void periodic(){
        PeriodicSubsystem.super.periodic();
        m_timeStamp = Timer.getFPGATimestamp();
    }

    @Override
    public synchronized void end(){
        leftMaster.stopMotor();
        rightMaster.stopMotor();
        leftSlave1.stopMotor();
        leftSlave2.stopMotor();
        rightSlave1.stopMotor();
        rightSlave2.stopMotor();
        periodicIO.leftOutput = 0;
        periodicIO.rightOutput = 0;
        periodicIO.leftFeedForward = 0;
        periodicIO.rightFeedForward = 0;
        controlType = ControlType.kDutyCycle;
    }

    public synchronized void setOpenLoop(DriveSignal driveSignal){
        if (driveSignal != null) {
            periodicIO.leftOutput = driveSignal.getLeft();
            periodicIO.rightOutput = driveSignal.getRight();
            periodicIO.leftFeedForward = 0;
            periodicIO.rightFeedForward = 0;
            controlType = ControlType.kDutyCycle;
        } else {
            end();
            HALMethods.sendDSError("DriveSignal in setOpenLoop was null");
        }
    }

    public void setVelocityMpS(double leftMPS, double rightMPS) {
        setVelocity(new DriveSignal(leftMPS, rightMPS));
    }

    public synchronized void setVelocity(DriveSignal driveSignal) {
        if (driveSignal != null) {
            
                periodicIO.leftFeedForward = feedforward.calculate(driveSignal.getLeft());
                periodicIO.rightFeedForward = feedforward.calculate(driveSignal.getRight());

                periodicIO.leftOutput = driveSignal.getLeft() / kEncoderVelocityToMetersPerSecond;
                periodicIO.rightOutput = driveSignal.getRight() / kEncoderVelocityToMetersPerSecond;
                controlType = ControlType.kVelocity;
            
        } else {
            HALMethods.sendDSError("Drive signal in setVelocity(DriveSignal) was null");
            end();
        }
    }

    public void setVolts(double leftVolts, double rightVolts) {
        setOpenLoop(new DriveSignal(leftVolts / 12, rightVolts / 12));
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean scaleInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .09);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .09);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        if (scaleInputs) {
            m_maxOutput = .7;
            leftMotorOutput *= m_maxOutput;
            rightMotorOutput *= m_maxOutput;
        }
        
        setOpenLoop(new DriveSignal(leftMotorOutput, rightMotorOutput));

        periodicIO.prevLeftDesiredVelocity = leftMotorOutput * m_leftPIDConfig.maxVelocity;
        periodicIO.prevRightDesiredVelocity = rightMotorOutput * m_rightPIDConfig.maxVelocity;
    }

    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }


    public synchronized void setToCoast() {
        SparkMaxUtil.checkError(leftMaster.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set left master to Coast mode");
        SparkMaxUtil.checkError(rightMaster.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set right master to Coast mode");

        SparkMaxUtil.checkError(leftSlave1.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set left slave 1 to Coast mode");
        SparkMaxUtil.checkError(rightSlave1.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set right slave 1 to Coast mode");
        SparkMaxUtil.checkError(leftSlave2.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set left slave 2 to Coast mode");
        SparkMaxUtil.checkError(rightSlave2.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set right slave 2 to Coast mode");
    }

    public synchronized void setToBrake() {
        SparkMaxUtil.checkError(leftMaster.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set left master to brake mode");
        SparkMaxUtil.checkError(rightMaster.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set right master to brake mode");


    }

    public double getLeftPosition() {
        return periodicIO.leftPosition;
    }

    public double getRightPosition() {
        return periodicIO.rightPosition;
    }

    public double getLeftVelocity() {
        return periodicIO.leftVelocity;
    }

    public double getRightVelocity() {
        return periodicIO.rightVelocity;
    }

    public boolean hasInitialized() {
        return initialized;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(periodicIO.leftVelocity, periodicIO.rightVelocity);
    }

    public double getAverageEncoderDistance() {
        return (periodicIO.leftPosition + periodicIO.rightPosition) / 2.0;
    }

    @Override
    public String getName() {
        return "Drivetrain";
        
    }

    



}