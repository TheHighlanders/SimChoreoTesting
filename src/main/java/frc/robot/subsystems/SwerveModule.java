package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.SwerveModuleConfig;

public class SwerveModule {
    public class Constants{
        static double kSDrive = 1;
        static double kVDrive = 1;
        static double kADrive = 1;
        static double kPDrive = 1;
        static double kIDrive = 0;
        static double kDDrive = 0;

        static double kPAngle = 1;
        static double kIAngle = 0;
        static double kDAngle = 0;

        public static final double kDriveGearRatio = 1.0f / 8.14f;
        public static final double kAngleGearRatio = 1.0f / 12.8f;

        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumfrence = kWheelDiameter * Math.PI;

        public static final double kDrivePositionConversionFactor = kDriveGearRatio * kWheelCircumfrence;
        public static final double kDriveVelocityConverstionFactor = kDrivePositionConversionFactor / 60.0f;

        public static final double kAnglePositionConversionFactor = kAngleGearRatio * 360.0;
        public static final double kAngleVelocityConverstionFactor = kAnglePositionConversionFactor / 60.0f;

        public static final boolean kAbsoluteEncoderInverted = false;
        public static final boolean kDriveInverted = false;
        public static final boolean kAngleInverted = false;

        public static final int kDriveCurrentLimit = 40;
        public static final int kAngleCurrentLimit = 20;
    }
    public CANSparkMax angleMotor;
    public CANSparkMax driveMotor;

    public int moduleNumber;

    public SparkPIDController driveController;
    public SparkPIDController angleController;

    private SimpleMotorFeedforward driveFeedforward;

    public RelativeEncoder driveEncoder;
    public RelativeEncoder angleEncoder;
    public double angleReference;
    public double driveReference;

    public SparkAbsoluteEncoder absoluteEncoder;

    private Rotation2d KModuleAbsoluteOffset;

    public SwerveModule(){}

    public SwerveModule(int moduleNumber, SwerveModuleConfig config) {
        this.moduleNumber = moduleNumber;

        driveMotor = new CANSparkMax(config.driveMotorID, MotorType.kBrushless);
        angleMotor = new CANSparkMax(config.angleMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = angleMotor.getEncoder();

        driveController = driveMotor.getPIDController();
        angleController = angleMotor.getPIDController();

        /* Creates an additional FF controller for extra drive motor control */
        driveFeedforward = new SimpleMotorFeedforward(Constants.kSDrive, Constants.kVDrive, Constants.kADrive);

        absoluteEncoder = angleMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        this.KModuleAbsoluteOffset = config.absoluteEncoderOffset;

        configureDriveMotor();
        configureAngleMotor();
    }

    /**
     * Sets both Angle and Drive to desired states
     *
     * @param state:      Desired module state
     * @param isOpenLoop: Controls if the drive motor use a PID loop
     */
    public void setModuleState(SwerveModuleState state, boolean isOpenLoop) {
        state = SwerveModuleState.optimize(state, getAnglePosition());

        setAngleState(state);
        setDriveState(state, isOpenLoop);
    }

    /**
     * Sets the Drive Motor to a desired state,
     * if isOpenLoop is true, it will be set as a percent, if it is false, than it
     * will use a velocity PIDF loop
     *
     * @param state:      Desired module state
     * @param isOpenLoop: Whether or not to use a PID loop
     */
    public void setDriveState(SwerveModuleState state, boolean isOpenLoop) {
        if (isOpenLoop) {
            double motorPercent = state.speedMetersPerSecond / Swerve.Constants.kMaxSpeed;
            driveMotor.set(motorPercent);
        } else {
            driveController.setReference(
                state.speedMetersPerSecond,
                ControlType.kVelocity,
                0,
                driveFeedforward.calculate(state.speedMetersPerSecond)
            );
            driveReference = state.speedMetersPerSecond;
        }
    }

    /**
     * Sets the Angle Motor to a desired state, does not set the state if speed is
     * too low, to stop wheel jitter
     *
     * @param state: Desired module state
     */
    public void setAngleState(SwerveModuleState state) {
        // Anti Jitter Code, not sure if it works, need to test and review
        // Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= SwerveConst.kMaxAngularSpeedFast * 0.001) ? lastAngle : state.angle;
        Rotation2d angle = state.angle;
        if (angle != null) {
            angleController.setReference(angle.getDegrees(), ControlType.kPosition);
            angleReference = angle.getDegrees();
        }
        // lastAngle = state.angle;
    }

    /**
     * Returns the position of the Angle Motor, measured with integrated encoder
     *
     * @return Angle Motor Position
     */
    public Rotation2d getAnglePosition() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    /**
     * Returns the velocity of the Drive Motor, measured with integrated encoder
     *
     * @return Drive Motor Velocity
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Gets the position of the Drive Motor, measured with integrated encoder
     *
     * @return Drive Motor Position
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Gets the position of the module using the absolute encoder
     *
     * @return Position of the module between 0 and 360, as a Rotation2d
     */
    public Rotation2d getAbsolutePosition() {
        /* Gets Position from SparkMAX absol encoder * 360 to degrees */
        // double positionDeg = avg * 360.0d;


        /* Gets Position from SparkMAX absol encoder * 360 to degrees */
        double positionDeg = absoluteEncoder.getPosition() * 360.0d;

        /* Subtracts magnetic offset to get wheel position */
        positionDeg -= KModuleAbsoluteOffset.getDegrees();

        /* Inverts if necesary */
        positionDeg *= (Constants.kAbsoluteEncoderInverted ? -1 : 1);

        return Rotation2d.fromDegrees(positionDeg);
    }

    public Rotation2d getAbsolutePositionNoOffset() {
        /* Gets Position from SparkMAX absol encoder * 360 to degrees */
        double positionDeg = absoluteEncoder.getPosition() * 360.0d;

        /* Inverts if necesary */
        positionDeg *= (Constants.kAbsoluteEncoderInverted ? -1 : 1);

        return Rotation2d.fromDegrees(positionDeg);
    }

    /**
     *
     * @return Swerve Module Position (Position & Angle)
     */
    public SwerveModulePosition getPosition() {
        DriverStation.reportWarning("REAL",false);
        return new SwerveModulePosition(-getDrivePosition(), getAnglePosition());
    }

    /**
     *
     * @return Swerve Module State (Velocity & Angle)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
    }

    /**
     * Returns the assigned module number
     */
    public int getModuleNumber() {
        return moduleNumber;
    }

    /**
     * Configures Drive Motor using parameters from Constants
     */
    public void configureDriveMotor() {
        driveMotor.restoreFactoryDefaults();

        driveMotor.setInverted(Constants.kDriveInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);

        /* Sets encoder ratios to actual module gear ratios */
        driveEncoder.setPositionConversionFactor(Constants.kDrivePositionConversionFactor);
        driveEncoder.setVelocityConversionFactor(Constants.kDriveVelocityConverstionFactor);

        /* Configures PID loop */
        driveController.setP(Constants.kPDrive);
        driveController.setI(Constants.kIDrive);
        driveController.setD(Constants.kDDrive);

        driveMotor.setSmartCurrentLimit(Constants.kDriveCurrentLimit);

        // driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    /**
     * Configures Angle Motor using parameters from Constants
     */
    public void configureAngleMotor() {
        angleMotor.restoreFactoryDefaults();

        angleMotor.setInverted(Constants.kAngleInverted);
        angleMotor.setIdleMode(IdleMode.kBrake);

        /* Sets encoder ratios to actual module gear ratios */
        angleEncoder.setPositionConversionFactor(Constants.kAnglePositionConversionFactor);
        angleEncoder.setVelocityConversionFactor(Constants.kAngleVelocityConverstionFactor);

        /* Configures PID loop */
        angleController.setP(Constants.kPAngle);
        angleController.setI(Constants.kIAngle);
        angleController.setD(Constants.kDAngle);

        /* Defines wheel angles as -pi to pi */
        angleController.setPositionPIDWrappingMaxInput(180.0d);
        angleController.setPositionPIDWrappingMinInput(-180.0d);
        angleController.setPositionPIDWrappingEnabled(true);

        angleMotor.setSmartCurrentLimit(Constants.kAngleCurrentLimit);

        // angleMotor.burnFlash();

        setIntegratedAngleToAbsolute();
    }

    /**
     * Resets the Angle Motor to the position of the absolute position
     */
    public void setIntegratedAngleToAbsolute() {
        angleEncoder.setPosition(getAbsolutePosition().getDegrees());
    }

    public void updateModule(){
        //No-op here, so Sim Module can extend seamlessly
        // This is bad, not sure of proper way
    }

    public SwerveModuleState getSetpoint(){
        return new SwerveModuleState(driveReference, Rotation2d.fromDegrees(angleReference)); 
    }
}
