package frc.robot.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleSim{
    public class Constants{
        static double kSDrive = 6.5;
        static double kVDrive = 0.5;

        static double kPDrive = 25;
        static double kIDrive = 0;
        static double kDDrive = 0;

        static double kPAngle = .1;
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
    private DCMotorSim angleMotor;
    private DCMotorSim driveMotor;

    public int moduleNumber;

    public SparkPIDController driveController;
    public SparkPIDController angleController;

    public SparkAbsoluteEncoder absoluteEncoder;
    
    public PIDController drivePID = new PIDController(Constants.kPDrive, Constants.kIDrive, Constants.kDDrive);
    public PIDController anglePID = new PIDController(Constants.kPAngle, Constants.kIAngle, Constants.kDAngle);
    public SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(Constants.kSDrive, Constants.kVDrive);

    public SwerveModuleSim(int moduleNumber) {
        this.moduleNumber = moduleNumber;

        driveMotor = new DCMotorSim(DCMotor.getNEO(1), 8.14, 0.025);
        angleMotor = new DCMotorSim(DCMotor.getNEO(1), 12.8, 0.004);
        anglePID.enableContinuousInput(-180, 180);
    }

    public void updateModule(){
        driveMotor.update(0.02);
        angleMotor.update(0.02);
    }

    /**
     * Returns the position of the Angle Motor, measured with integrated encoder
     *
     * @return Angle Motor Position
     */
    public Rotation2d getAnglePosition() {
        return new Rotation2d(angleMotor.getAngularPositionRad());
    }

    public Rotation2d getAngleSetpoint() {
        return Rotation2d.fromDegrees(anglePID.getSetpoint());
    }

    /**
     * Returns the velocity of the Drive Motor, measured with integrated encoder
     *
     * @return Drive Motor Velocity
     */
    public double getDriveVelocity() {
        return driveMotor.getAngularVelocityRPM() * (Math.PI * Units.inchesToMeters(4)) * (1/60.0);
    }

    /**
     * Gets the position of the Drive Motor, measured with integrated encoder
     *
     * @return Drive Motor Position
     */
    public double getDrivePosition() {
        return driveMotor.getAngularPositionRotations() * (Math.PI * Units.inchesToMeters(4));
    }
    public double getDriveSetpoint(){
        return drivePID.getSetpoint();
    }

    /**
     *
     * @return Swerve Module Position (Position & Angle)
     */
    public SwerveModulePosition getPosition() {
        SwerveModulePosition p = new SwerveModulePosition(-getDrivePosition(), getAnglePosition());
        return p;
    }

    /**
     *
     * @return Swerve Module State (Velocity & Angle)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
    }

    public void setModuleState(SwerveModuleState state){
        setDriveState(state);
        setAngleState(state);
    }

    public void setDriveState(SwerveModuleState state){
        driveMotor.setInput(MathUtil.clamp(drivePID.calculate(getDriveVelocity(),state.speedMetersPerSecond) + driveFF.calculate(state.speedMetersPerSecond), -12.0, 12.0));
    }

    public void setAngleState(SwerveModuleState state){
        angleMotor.setInput(MathUtil.clamp(anglePID.calculate(getAnglePosition().getDegrees(), state.angle.getDegrees()), -12.0, 12.0));
    }

    /**
     * Returns the assigned module number
     */
    public int getModuleNumber() {
        return moduleNumber;
    }
}
