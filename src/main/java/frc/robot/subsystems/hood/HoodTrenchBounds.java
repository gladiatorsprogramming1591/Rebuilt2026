// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.hood;

import frc.robot.util.FieldConstants;
import frc.robot.util.Bounds;

public class HoodTrenchBounds {
  public static final Bounds blueNearTrench =
      new Bounds(
          FieldConstants.LinesVertical.hubCenter - 0.61,
          FieldConstants.LinesVertical.hubCenter + 1.61,
          FieldConstants.LinesHorizontal.rightTrenchOpenEnd,
          FieldConstants.LinesHorizontal.rightTrenchOpenStart);

  public static final Bounds blueFarTrench =
      new Bounds(
          FieldConstants.LinesVertical.hubCenter - 0.61,
          FieldConstants.LinesVertical.hubCenter + 1.61,
          FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
          FieldConstants.LinesHorizontal.leftTrenchOpenStart);

  public static final Bounds redNearTrench =
      new Bounds(
          FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter - 1.61,
          FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter + 0.61,
          FieldConstants.LinesHorizontal.rightTrenchOpenEnd,
          FieldConstants.LinesHorizontal.rightTrenchOpenStart);

  public static final Bounds redFarTrench =
      new Bounds(
          FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter - 1.61,
          FieldConstants.fieldLength - FieldConstants.LinesVertical.hubCenter + 0.61,
          FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
          FieldConstants.LinesHorizontal.leftTrenchOpenStart);
}
