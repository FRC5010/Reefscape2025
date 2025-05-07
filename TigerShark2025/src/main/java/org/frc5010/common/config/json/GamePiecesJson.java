// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.crescendo2024.CrescendoNoteOnField;

import edu.wpi.first.math.geometry.Translation2d;

/** Json class for game piece configurations */
public class GamePiecesJson {
    public GamePieceJson[] gamePieces;

    public void createGamePieces(GenericSwerveDrivetrain drivetrain) {
        SimulatedArena arena = SimulatedArena.getInstance();
        for (GamePieceJson gamePiece : gamePieces) {
            switch (gamePiece.type) {
                case "Note":
                    CrescendoNoteOnField note = new CrescendoNoteOnField(new Translation2d(gamePiece.x, gamePiece.y));
                    arena.addGamePiece(note);
                    break;
                default:
                    break;
            }
        }
    }
}
