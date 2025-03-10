package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX; // Kraken X60 motors use TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.*; // Needed for setting motor control modes
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ElevatorFella extends SubsystemBase {
    private final TalonFX elevatorMotor1; // First Kraken X60 motor
    private final TalonFX elevatorMotor2; // Second Kraken X60 motor
    private final SlewRateLimiter elevatorlimiter = new SlewRateLimiter(1.5);

    public ElevatorFella() {
        elevatorMotor1 = new TalonFX(20); // Replace with your actual CAN ID
        elevatorMotor2 = new TalonFX(21); // Replace with your actual CAN ID
        
       elevatorMotor1.setNeutralMode(NeutralModeValue.Brake); // Set the brake mode
       elevatorMotor2.setNeutralMode(NeutralModeValue.Brake); //Set the brake mode
    }

    /**
     * Sets the elevator speed (positive = up, negative = down).
     * @param speed -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        double holdVoltage = -.315  ; //Positin Holding Power
        double elevatorDeadband = 0.02; //Ignores small values of stuff so it aint tweak out
        if (Math.abs(speed) < elevatorDeadband) {
            elevatorMotor1.setVoltage(holdVoltage);
            elevatorMotor2.setVoltage(holdVoltage);
        } else {
            elevatorMotor1.setControl(new DutyCycleOut(speed));
            elevatorMotor2.setControl(new DutyCycleOut(speed));
        }
       
    }

    /** Stops the elevator */
    public void stop() {
        setSpeed(0);
    }





  public class MoveElevator extends Command {
        private final ElevatorFella elevator;
        private final double speed;
    
        public MoveElevator(ElevatorFella elevator, double speed) {
            double limitedSpeed = elevatorlimiter.calculate(speed);
            elevatorMotor1.set(limitedSpeed);
            elevatorMotor2.set(limitedSpeed);
            this.elevator = elevator;
            this.speed = speed;
            addRequirements(elevator); // Ensures only one command controls the elevator at a time
        }
    
        @Override
        public void execute() {
            elevator.setSpeed(speed);
        }
    
        @Override
        public void end(boolean interrupted) {
            elevator.stop(); // Stops the elevator when the command ends
        }
    
        @Override
        public boolean isFinished() {
            return false; // Keeps running until button is released
        }
    }
    



}