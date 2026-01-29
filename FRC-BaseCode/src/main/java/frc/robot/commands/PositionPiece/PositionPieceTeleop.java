package frc.robot.commands.PositionPiece;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.positionPiece.PositionPieceSubsystem;

public class PositionPieceTeleop extends Command {
    private final PositionPieceSubsystem piece;

    private final Supplier<Double> manualAxis;
    private final double kManualMaxVolts;

    public PositionPieceTeleop(
        PositionPieceSubsystem piece,
        Supplier<Double> manualAxis,
        double kManualMaxVolts
    ) {
        this.piece = piece;

        this.manualAxis = manualAxis;
        this.kManualMaxVolts = kManualMaxVolts;

        addRequirements(piece);
    }

    @Override
    public void execute() {
        if (manualAxis.get() != 0) {
            double axis = manualAxis.get();
            piece.setManualVolts(axis * kManualMaxVolts);
            return;
        }

        //Idle
        piece.setManualVolts(0.0);
    }

    @Override
    public void end(boolean interrupted) {
        piece.stop();
    }
}
