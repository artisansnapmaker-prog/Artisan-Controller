/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)

#include "../bedlevel.h"

#include "../../../module/motion.h"

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

#include "../../../module/planner.h"

float startx = DOUBLE_EXTRUDER_X_BILINEAR_START_POINT;
float endx   = DOUBLE_EXTRUDER_X_BILINEAR_END_POINT;
float starty = DOUBLE_EXTRUDER_Y_BILINEAR_START_POINT;
float endy   = DOUBLE_EXTRUDER_Y_BILINEAR_END_POINT;

xy_pos_t bilinear_grid_spacing, bilinear_start;
xy_float_t bilinear_grid_factor;
bed_mesh_t z_values;
bed_mesh_t z_values_raw;

/**
 * Extrapolate a single point from its neighbors
 */
static void extrapolate_one_point(const uint8_t x, const uint8_t y, const int8_t xdir, const int8_t ydir) {
  if (!isnan(z_values[x][y])) return;
  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOPGM("Extrapolate [");
    if (x < 10) DEBUG_CHAR(' ');
    DEBUG_ECHO(x);
    DEBUG_CHAR(xdir ? (xdir > 0 ? '+' : '-') : ' ');
    DEBUG_CHAR(' ');
    if (y < 10) DEBUG_CHAR(' ');
    DEBUG_ECHO(y);
    DEBUG_CHAR(ydir ? (ydir > 0 ? '+' : '-') : ' ');
    DEBUG_ECHOLNPGM("]");
  }

  // Get X neighbors, Y neighbors, and XY neighbors
  const uint8_t x1 = x + xdir, y1 = y + ydir, x2 = x1 + xdir, y2 = y1 + ydir;
  float a1 = z_values[x1][y ], a2 = z_values[x2][y ],
        b1 = z_values[x ][y1], b2 = z_values[x ][y2],
        c1 = z_values[x1][y1], c2 = z_values[x2][y2];

  // Treat far unprobed points as zero, near as equal to far
  if (isnan(a2)) a2 = 0.0;
  if (isnan(a1)) a1 = a2;
  if (isnan(b2)) b2 = 0.0;
  if (isnan(b1)) b1 = b2;
  if (isnan(c2)) c2 = 0.0;
  if (isnan(c1)) c1 = c2;

  const float a = 2 * a1 - a2, b = 2 * b1 - b2, c = 2 * c1 - c2;

  // Take the average instead of the median
  z_values[x][y] = (a + b + c) / 3.0;
  TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(x, y, z_values[x][y]));

  // Median is robust (ignores outliers).
  // z_values[x][y] = (a < b) ? ((b < c) ? b : (c < a) ? a : c)
  //                                : ((c < b) ? b : (a < c) ? a : c);
}

//Enable this if your SCARA uses 180° of total area
//#define EXTRAPOLATE_FROM_EDGE

#if ENABLED(EXTRAPOLATE_FROM_EDGE)
  #if (GRID_MAX_POINTS_X) < (GRID_MAX_POINTS_Y)
    #define HALF_IN_X
  #elif (GRID_MAX_POINTS_Y) < (GRID_MAX_POINTS_X)
    #define HALF_IN_Y
  #endif
#endif

/**
 * Fill in the unprobed points (corners of circular print surface)
 * using linear extrapolation, away from the center.
 */
void extrapolate_unprobed_bed_level() {
  #ifdef HALF_IN_X
    constexpr uint8_t ctrx2 = 0, xend = GRID_MAX_POINTS_X - 1;
  #else
   uint8_t ctrx1 = (GRID_MAX_CELLS_X) / 2, // left-of-center
                      ctrx2 = (GRID_MAX_POINTS_X) / 2,  // right-of-center
                      xend = ctrx1;
  #endif

  #ifdef HALF_IN_Y
    constexpr uint8_t ctry2 = 0, yend = GRID_MAX_POINTS_Y - 1;
  #else
    uint8_t ctry1 = (GRID_MAX_CELLS_Y) / 2, // top-of-center
                      ctry2 = (GRID_MAX_POINTS_Y) / 2,  // bottom-of-center
                      yend = ctry1;
  #endif

  LOOP_LE_N(xo, xend)
    LOOP_LE_N(yo, yend) {
      uint8_t x2 = ctrx2 + xo, y2 = ctry2 + yo;
      #ifndef HALF_IN_X
        const uint8_t x1 = ctrx1 - xo;
      #endif
      #ifndef HALF_IN_Y
        const uint8_t y1 = ctry1 - yo;
        #ifndef HALF_IN_X
          extrapolate_one_point(x1, y1, +1, +1);   //  left-below + +
        #endif
        extrapolate_one_point(x2, y1, -1, +1);     // right-below - +
      #endif
      #ifndef HALF_IN_X
        extrapolate_one_point(x1, y2, +1, -1);     //  left-above + -
      #endif
      extrapolate_one_point(x2, y2, -1, -1);       // right-above - -
    }

}

void print_bilinear_leveling_grid() {
  SERIAL_ECHOLNPGM("Raw Bilinear Leveling Grid:");
  print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
    [](const uint8_t ix, const uint8_t iy) { return z_values_raw[ix][iy]; }
  );
  SERIAL_ECHOLNPGM("compensated Bilinear Leveling Grid:");
  print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
    [](const uint8_t ix, const uint8_t iy) { return z_values[ix][iy]; }
  );
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)

  #define ABL_GRID_POINTS_VIRT_X GRID_MAX_CELLS_X * (BILINEAR_SUBDIVISIONS) + 1
  #define ABL_GRID_POINTS_VIRT_Y GRID_MAX_CELLS_Y * (BILINEAR_SUBDIVISIONS) + 1
  #define ABL_TEMP_POINTS_X (GRID_MAX_POINTS_X + 2)
  #define ABL_TEMP_POINTS_Y (GRID_MAX_POINTS_Y + 2)

  float z_values_virt[VIRTUAL_GRID_MAX_NUM][VIRTUAL_GRID_MAX_NUM];
  xy_pos_t bilinear_grid_spacing_virt;
  xy_float_t bilinear_grid_factor_virt;

  void print_bilinear_leveling_grid_virt() {
    SERIAL_ECHOLNPGM("Subdivided with CATMULL ROM Leveling Grid:");
    print_2d_array(ABL_GRID_POINTS_VIRT_X, ABL_GRID_POINTS_VIRT_Y, 5,
      [](const uint8_t ix, const uint8_t iy) { return z_values_virt[ix][iy]; }
    );
  }

  #define LINEAR_EXTRAPOLATION(E, I) ((E) * 2 - (I))
  float bed_level_virt_coord(const uint8_t x, const uint8_t y) {
    uint8_t ep = 0, ip = 1;
    if (x > (GRID_MAX_POINTS_X) + 1 || y > (GRID_MAX_POINTS_Y) + 1) {
      // The requested point requires extrapolating two points beyond the mesh.
      // These values are only requested for the edges of the mesh, which are always an actual mesh point,
      // and do not require interpolation. When interpolation is not needed, this "Mesh + 2" point is
      // cancelled out in bed_level_virt_cmr and does not impact the result. Return 0.0 rather than
      // making this function more complex by extrapolating two points.
      return 0.0;
    }
    if (!x || x == ABL_TEMP_POINTS_X - 1) {
      if (x) {
        ep = (GRID_MAX_POINTS_X) - 1;
        ip = GRID_MAX_CELLS_X - 1;
      }
      if (WITHIN(y, 1, ABL_TEMP_POINTS_Y - 2))
        return LINEAR_EXTRAPOLATION(
          z_values[ep][y - 1],
          z_values[ip][y - 1]
        );
      else
        return LINEAR_EXTRAPOLATION(
          bed_level_virt_coord(ep + 1, y),
          bed_level_virt_coord(ip + 1, y)
        );
    }
    if (!y || y == ABL_TEMP_POINTS_Y - 1) {
      if (y) {
        ep = (GRID_MAX_POINTS_Y) - 1;
        ip = GRID_MAX_CELLS_Y - 1;
      }
      if (WITHIN(x, 1, ABL_TEMP_POINTS_X - 2))
        return LINEAR_EXTRAPOLATION(
          z_values[x - 1][ep],
          z_values[x - 1][ip]
        );
      else
        return LINEAR_EXTRAPOLATION(
          bed_level_virt_coord(x, ep + 1),
          bed_level_virt_coord(x, ip + 1)
        );
    }
    return z_values[x - 1][y - 1];
  }

  static float bed_level_virt_cmr(const float p[4], const uint8_t i, const float t) {
    return (
        p[i-1] * -t * sq(1 - t)
      + p[i]   * (2 - 5 * sq(t) + 3 * t * sq(t))
      + p[i+1] * t * (1 + 4 * t - 3 * sq(t))
      - p[i+2] * sq(t) * (1 - t)
    ) * 0.5f;
  }

  static float bed_level_virt_2cmr(const uint8_t x, const uint8_t y, const_float_t tx, const_float_t ty) {
    float row[4], column[4];
    LOOP_L_N(i, 4) {
      LOOP_L_N(j, 4) {
        column[j] = bed_level_virt_coord(i + x - 1, j + y - 1);
      }
      row[i] = bed_level_virt_cmr(column, 1, ty);
    }
    return bed_level_virt_cmr(row, 1, tx);
  }

  void bed_level_virt_interpolate() {
    bilinear_grid_spacing_virt = bilinear_grid_spacing / (BILINEAR_SUBDIVISIONS);
    bilinear_grid_factor_virt = bilinear_grid_spacing_virt.reciprocal();
    LOOP_L_N(y, GRID_MAX_POINTS_Y)
      LOOP_L_N(x, GRID_MAX_POINTS_X)
        LOOP_L_N(ty, BILINEAR_SUBDIVISIONS)
          LOOP_L_N(tx, BILINEAR_SUBDIVISIONS) {
            if ((ty && y == (GRID_MAX_POINTS_Y) - 1) || (tx && x == (GRID_MAX_POINTS_X) - 1))
              continue;
            z_values_virt[x * (BILINEAR_SUBDIVISIONS) + tx][y * (BILINEAR_SUBDIVISIONS) + ty] =
              bed_level_virt_2cmr(
                x + 1,
                y + 1,
                (float)tx / (BILINEAR_SUBDIVISIONS),
                (float)ty / (BILINEAR_SUBDIVISIONS)
              );
          }
  }
#endif // ABL_BILINEAR_SUBDIVISION

// Refresh after other values have been updated
void refresh_bed_level() {
  bilinear_grid_factor = bilinear_grid_spacing.reciprocal();
  TERN_(ABL_BILINEAR_SUBDIVISION, bed_level_virt_interpolate());
}

#if ENABLED(ABL_BILINEAR_SUBDIVISION)
  #define ABL_BG_SPACING(A) bilinear_grid_spacing_virt.A
  #define ABL_BG_FACTOR(A)  bilinear_grid_factor_virt.A
  #define ABL_BG_POINTS_X   ABL_GRID_POINTS_VIRT_X
  #define ABL_BG_POINTS_Y   ABL_GRID_POINTS_VIRT_Y
  #define ABL_BG_GRID(X,Y)  z_values_virt[X][Y]
#else
  #define ABL_BG_SPACING(A) bilinear_grid_spacing.A
  #define ABL_BG_FACTOR(A)  bilinear_grid_factor.A
  #define ABL_BG_POINTS_X   GRID_MAX_POINTS_X
  #define ABL_BG_POINTS_Y   GRID_MAX_POINTS_Y
  #define ABL_BG_GRID(X,Y)  z_values[X][Y]
#endif

// Get the Z adjustment for non-linear bed leveling
float bilinear_z_offset(const xy_pos_t &raw) {

  static float z1, d2, z3, d4, L, D;

  static xy_pos_t prev { -999.999, -999.999 }, ratio;

  // Whole units for the grid line indices. Constrained within bounds.
  static xy_int8_t thisg, nextg, lastg { -99, -99 };

  // XY relative to the probed area
  xy_pos_t rel = raw - bilinear_start.asFloat();

  #if ENABLED(EXTRAPOLATE_BEYOND_GRID)
    #define FAR_EDGE_OR_BOX 2   // Keep using the last grid box
  #else
    #define FAR_EDGE_OR_BOX 1   // Just use the grid far edge
  #endif

  if (prev.x != rel.x) {
    prev.x = rel.x;
    ratio.x = rel.x * ABL_BG_FACTOR(x);
    const float gx = constrain(FLOOR(ratio.x), 0, ABL_BG_POINTS_X - (FAR_EDGE_OR_BOX));
    ratio.x -= gx;      // Subtract whole to get the ratio within the grid box

    #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
      // Beyond the grid maintain height at grid edges
      NOLESS(ratio.x, 0); // Never <0 (>1 is ok when nextg.x==thisg.x)
    #endif

    thisg.x = gx;
    nextg.x = _MIN(thisg.x + 1, ABL_BG_POINTS_X - 1);
  }

  if (prev.y != rel.y || lastg.x != thisg.x) {

    if (prev.y != rel.y) {
      prev.y = rel.y;
      ratio.y = rel.y * ABL_BG_FACTOR(y);
      const float gy = constrain(FLOOR(ratio.y), 0, ABL_BG_POINTS_Y - (FAR_EDGE_OR_BOX));
      ratio.y -= gy;

      #if DISABLED(EXTRAPOLATE_BEYOND_GRID)
        // Beyond the grid maintain height at grid edges
        NOLESS(ratio.y, 0); // Never < 0.0. (> 1.0 is ok when nextg.y==thisg.y.)
      #endif

      thisg.y = gy;
      nextg.y = _MIN(thisg.y + 1, ABL_BG_POINTS_Y - 1);
    }

    if (lastg != thisg) {
      lastg = thisg;
      // Z at the box corners
      z1 = ABL_BG_GRID(thisg.x, thisg.y);       // left-front
      d2 = ABL_BG_GRID(thisg.x, nextg.y) - z1;  // left-back (delta)
      z3 = ABL_BG_GRID(nextg.x, thisg.y);       // right-front
      d4 = ABL_BG_GRID(nextg.x, nextg.y) - z3;  // right-back (delta)
    }

    // Bilinear interpolate. Needed since rel.y or thisg.x has changed.
                L = z1 + d2 * ratio.y;   // Linear interp. LF -> LB
    const float R = z3 + d4 * ratio.y;   // Linear interp. RF -> RB

    D = R - L;
  }

  const float offset = L + ratio.x * D;   // the offset almost always changes

  /*
  static float last_offset = 0;
  if (ABS(last_offset - offset) > 0.2) {
    SERIAL_ECHOLNPGM("Sudden Shift at x=", rel.x, " / ", bilinear_grid_spacing.x, " -> thisg.x=", thisg.x);
    SERIAL_ECHOLNPGM(" y=", rel.y, " / ", bilinear_grid_spacing.y, " -> thisg.y=", thisg.y);
    SERIAL_ECHOLNPGM(" ratio.x=", ratio.x, " ratio.y=", ratio.y);
    SERIAL_ECHOLNPGM(" z1=", z1, " z2=", z2, " z3=", z3, " z4=", z4);
    SERIAL_ECHOLNPGM(" L=", L, " R=", R, " offset=", offset);
  }
  last_offset = offset;
  //*/

  return offset;
}

#if IS_CARTESIAN && DISABLED(SEGMENT_LEVELED_MOVES)

  #define CELL_INDEX(A,V) ((V - bilinear_start.A) * ABL_BG_FACTOR(A))

  /**
   * Prepare a bilinear-leveled linear move on Cartesian,
   * splitting the move where it crosses grid borders.
   */
#if DISABLED(CUSTOM_SEGMENT_LEVELED_MOVES)
  void bilinear_line_to_destination(const_feedRate_t scaled_fr_mm_s, uint16_t x_splits, uint16_t y_splits) {
    // Get current and destination cells for this line
    xy_int_t c1 { (short int)CELL_INDEX(x, current_position.x), (short int)CELL_INDEX(y, current_position.y) },
             c2 { (short int)CELL_INDEX(x, destination.x), (short int)CELL_INDEX(y, destination.y) };
    LIMIT(c1.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c1.y, 0, ABL_BG_POINTS_Y - 2);
    LIMIT(c2.x, 0, ABL_BG_POINTS_X - 2);
    LIMIT(c2.y, 0, ABL_BG_POINTS_Y - 2);

    // Start and end in the same cell? No split needed.
    if (c1 == c2) {
      current_position = destination;
      line_to_current_position(scaled_fr_mm_s);
      return;
    }

    #define LINE_SEGMENT_END(A) (current_position.A + (destination.A - current_position.A) * normalized_dist)

    float normalized_dist;
    xyze_pos_t end;
    const xy_int8_t gc { (signed char)_MAX(c1.x, c2.x), (signed char)_MAX(c1.y, c2.y) };

    // Crosses on the X and not already split on this X?
    // The x_splits flags are insurance against rounding errors.
    if (c2.x != c1.x && TEST(x_splits, gc.x)) {
      // Split on the X grid line
      CBI(x_splits, gc.x);
      end = destination;
      destination.x = bilinear_start.x + ABL_BG_SPACING(x) * gc.x;
      normalized_dist = (destination.x - current_position.x) / (end.x - current_position.x);
      destination.y = LINE_SEGMENT_END(y);
    }
    // Crosses on the Y and not already split on this Y?
    else if (c2.y != c1.y && TEST(y_splits, gc.y)) {
      // Split on the Y grid line
      CBI(y_splits, gc.y);
      end = destination;
      destination.y = bilinear_start.y + ABL_BG_SPACING(y) * gc.y;
      normalized_dist = (destination.y - current_position.y) / (end.y - current_position.y);
      destination.x = LINE_SEGMENT_END(x);
    }
    else {
      // Must already have been split on these border(s)
      // This should be a rare case.
      current_position = destination;
      line_to_current_position(scaled_fr_mm_s);
      return;
    }

    destination.z = LINE_SEGMENT_END(z);
    destination.e = LINE_SEGMENT_END(e);

    // Do the split and look for more borders
    bilinear_line_to_destination(scaled_fr_mm_s, x_splits, y_splits);

    // Restore destination from stack
    destination = end;
    bilinear_line_to_destination(scaled_fr_mm_s, x_splits, y_splits);
  }

#else

  #define  MIN_VALID_SEGMENT_LENGTH                   5.0   //mm
  #define  CAL_LINE_SEGMENT_END(start, end, scale)    (start + (end - start) * scale)

  static float mesh_index_to_xpos(int16_t i) {
    LIMIT(i, 0, ABL_BG_POINTS_X - 1);
    return bilinear_start.x + ABL_BG_SPACING(x) * i;
  }

  static float mesh_index_to_ypos(int16_t i) {
    LIMIT(i, 0, ABL_BG_POINTS_Y - 1);
    return bilinear_start.y + ABL_BG_SPACING(y) * i;
  }

  void bilinear_line_to_destination(const_feedRate_t scaled_fr_mm_s, uint16_t x_splits, uint16_t y_splits) {
    const xyze_pos_t &start = current_position, &end = destination;
    const xyze_pos_t dist = end - start;
    // Get current and destination cells for this line
    xy_int_t istart { (short int)CELL_INDEX(x, current_position.x), (short int)CELL_INDEX(y, current_position.y) },
             iend { (short int)CELL_INDEX(x, destination.x), (short int)CELL_INDEX(y, destination.y) };
    LIMIT(istart.x, 0, ABL_BG_POINTS_X - 1);
    LIMIT(istart.y, 0, ABL_BG_POINTS_Y - 1);
    LIMIT(iend.x, 0, ABL_BG_POINTS_X - 1);
    LIMIT(iend.y, 0, ABL_BG_POINTS_Y - 1);

    extern Planner planner;
    // LOG_I("start.x: %f start.y:%f, end.x: %f end.y : %f\n", start.x, start.y, end.x, end.y);
    // LOG_I("istart.x: %d istart.y:%d, iend.x: %d iend.y : %d\n", istart.x, istart.y, iend.x, iend.y);
    // A move within the same cell needs no splitting
    if ((!dist.x && !dist.y) || !planner.leveling_active || istart == iend) {

      FINAL_MOVE:

      planner.buffer_line(destination, scaled_fr_mm_s);
      // LOG_I("FINAL_MOVE, istart.x: %d, istart.y: %d, end.x: %f end.y : %f, end.e: %f end.i: %f, end.j: %f\n", istart.x, istart.y, end.x, end.y, end.e, end.i, end.j);
      current_position = destination;
      return;
    }

    const xy_bool_t neg { dist.x < 0, dist.y < 0 };
    const xy_int_t ineg { int8_t(neg.x), int8_t(neg.y) };
    const xy_float_t sign { neg.x ? -1.0f : 1.0f, neg.y ? -1.0f : 1.0f };
    const xy_int_t iadd { int8_t(iend.x == istart.x ? 0 : sign.x), int8_t(iend.y == istart.y ? 0 : sign.y) };
    const xy_float_t ad = sign * dist;
    const bool use_x_dist = ad.x > ad.y;
    // float on_axis_distance = use_x_dist ? dist.x : dist.y;
    xy_int_t icell = istart;
    const float ratio = dist.y / dist.x,        // Allow divide by zero
              c = start.y - ratio * start.x;
    const bool inf_ratio_flag = isinf(ratio);
    xyze_pos_t dest = start;
    xyze_pos_t cur_dest = start;
    // xyze_int8_t dist_need_move = {  (int8_t)(dist.x == 0 ? 0 : 1),
    //                                 (int8_t)(dist.y == 0 ? 0 : 1),
    //                                 (int8_t)(dist.z == 0 ? 0 : 1),
    //                               #if HAS_EXTRUDERS
    //                                 (int8_t)(dist.e == 0 ? 0 : 1),
    //                               #endif
    //                               #if LINEAR_AXES >= 4
    //                                 (int8_t)(dist.i == 0 ? 0 : 1),
    //                               #endif
    //                               #if LINEAR_AXES >= 5
    //                                 (int8_t)(dist.j == 0 ? 0 : 1),
    //                               #endif
    //                              };

    float normalized_dist = 0;
    float min_valid_segment_len_sqr = MIN_VALID_SEGMENT_LENGTH * MIN_VALID_SEGMENT_LENGTH;

    if (iadd.x == 0) {
      icell.y += ineg.y;
      while (icell.y != iend.y + ineg.y) {
        icell.y += iadd.y;
        const float next_mesh_line_y = mesh_index_to_ypos(icell.y);
        dest.x = inf_ratio_flag ? start.x : (next_mesh_line_y - c) / ratio;
        dest.y = mesh_index_to_ypos(icell.y);

        if (dest.y != start.y) {
          normalized_dist = (dest.y - start.y) / (end.y - start.y);
          dest.z = CAL_LINE_SEGMENT_END(start.z, end.z, normalized_dist);
          TERN_(LINEAR_AXES >= 4, dest.i = CAL_LINE_SEGMENT_END(start.i, end.i, normalized_dist));
          TERN_(LINEAR_AXES >= 5, dest.j = CAL_LINE_SEGMENT_END(start.j, end.j, normalized_dist));
          TERN_(HAS_EXTRUDERS, dest.e = CAL_LINE_SEGMENT_END(start.e, end.e, normalized_dist));
          // planner.buffer_line(dest, scaled_fr_mm_s);
          if (((dest.x - cur_dest.x)*(dest.x - cur_dest.x) + (dest.y - cur_dest.y) * (dest.y - cur_dest.y)) >= min_valid_segment_len_sqr) {
            // LOG_I("icell.x: %d, icell.y: %d\n", icell.x, icell.y);
            // LOG_I("start.x: %f, start.y: %f, end.x: %f, end.y: %f, k: %f, ratio: %f, inf_ratio_flag: %d\n", cur_dest.x, cur_dest.y, dest.x, dest.y, (dest.y - cur_dest.y)/(dest.x - cur_dest.x), ratio, inf_ratio_flag);
            cur_dest = dest;
            if (!planner.buffer_line(dest, scaled_fr_mm_s)) break;
          }
          // LOG_I("dest.y != start.y\n");
          // LOG_I("dest.y != start.y, istart.x: %d, istart.y: %d, end.x: %f end.y : %f end.e: %f end.i: %f, end.j: %f\n", icell.x, icell.y, dest.x, dest.y, dest.e, dest.i, dest.j);
          // if (dest.i != 0 || dest.i != 0) {
          //   LOG_I("\n\ndest.i: %f, dest.j: %f, normalized_dist: %f start.i: %f, start.j: %f\n", dest.i, dest.j, normalized_dist, start.i, start.j);
          // }
        }
      }

      if (dest != end)
        goto FINAL_MOVE;

      current_position = destination;
      return;
    }

    if (iadd.y == 0) {
      icell.x += ineg.x;

      while (icell.x != iend.x + ineg.x) {
        icell.x += iadd.x;
        dest.x = mesh_index_to_xpos(icell.x);
        dest.y = ratio * dest.x + c;

        if (dest.x != start.x) {
          normalized_dist = (dest.x - start.x) / (end.x - start.x);
          dest.z = CAL_LINE_SEGMENT_END(start.z, end.z, normalized_dist);
          TERN_(LINEAR_AXES >= 4, dest.i = CAL_LINE_SEGMENT_END(start.i, end.i, normalized_dist));
          TERN_(LINEAR_AXES >= 5, dest.j = CAL_LINE_SEGMENT_END(start.j, end.j, normalized_dist));
          TERN_(HAS_EXTRUDERS, dest.e = CAL_LINE_SEGMENT_END(start.e, end.e, normalized_dist));
          // planner.buffer_line(dest, scaled_fr_mm_s);
          if (((dest.x - cur_dest.x)*(dest.x - cur_dest.x) + (dest.y - cur_dest.y) * (dest.y - cur_dest.y)) >= min_valid_segment_len_sqr) {
            // LOG_I("start.x: %f, start.y: %f, end.x: %f, end.y: %f, k: %f, ratio: %f, inf_ratio_flag: %d\n", cur_dest.x, cur_dest.y, dest.x, dest.y, (dest.y - cur_dest.y)/(dest.x - cur_dest.x), ratio, inf_ratio_flag);
            cur_dest = dest;
            // LOG_I("icell.x: %d, icell.y: %d\n", icell.x, icell.y);
            if (!planner.buffer_line(dest, scaled_fr_mm_s)) break;
          }
          // if (dest.i != 0 || dest.i != 0) {
          //   LOG_I("\n\ndest.i: %f, dest.j: %f, normalized_dist: %f start.i: %f, start.j: %f\n", dest.i, dest.j, normalized_dist, start.i, start.j);
          // }
          // LOG_I("dest.x != start.x\n");
          // LOG_I("dest.x != start.x, istart.x: %d, istart.y: %d, end.x: %f end.y : %f end.e: %f end.i: %f, end.j: %f\n", icell.x, icell.y, dest.x, dest.y, dest.e, dest.i, dest.j);
        }
      }

      if (dest != end)
        goto FINAL_MOVE;

      current_position = destination;
      return;
    }

    xy_int_t cnt = (istart - iend).ABS();
    icell += ineg;

    while (cnt) {

      const float next_mesh_line_x = mesh_index_to_xpos(icell.x + iadd.x),
                  next_mesh_line_y = mesh_index_to_ypos(icell.y + iadd.y);

      dest.y = ratio * next_mesh_line_x + c;
      dest.x = (next_mesh_line_y - c) / ratio;

      if (neg.x == (dest.x > next_mesh_line_x)) { // Check if we hit the Y line first
        dest.y = next_mesh_line_y;
        if (use_x_dist)
          normalized_dist = (dest.x - start.x) / (end.x - start.x);
        else
          normalized_dist = (dest.y - start.y) / (end.y - start.y);
        // dest.y = CAL_LINE_SEGMENT_END(start.y, end.y, normalized_dist);
        dest.z = CAL_LINE_SEGMENT_END(start.z, end.z, normalized_dist);
        TERN_(LINEAR_AXES >= 4, dest.i = CAL_LINE_SEGMENT_END(start.i, end.i, normalized_dist));
        TERN_(LINEAR_AXES >= 5, dest.j = CAL_LINE_SEGMENT_END(start.j, end.j, normalized_dist));
        TERN_(HAS_EXTRUDERS, dest.e = CAL_LINE_SEGMENT_END(start.e, end.e, normalized_dist));
        // planner.buffer_line(dest, scaled_fr_mm_s);
        if (((dest.x - cur_dest.x)*(dest.x - cur_dest.x) + (dest.y - cur_dest.y) * (dest.y - cur_dest.y)) >= min_valid_segment_len_sqr) {
          // LOG_I("start.x: %f, start.y: %f, end.x: %f, end.y: %f, k: %f, ratio: %f, inf_ratio_flag: %d\n", cur_dest.x, cur_dest.y, dest.x, dest.y, (dest.y - cur_dest.y)/(dest.x - cur_dest.x), ratio, inf_ratio_flag);
          cur_dest = dest;
          // LOG_I("icell.x: %d, icell.y: %d\n", icell.x, icell.y);
          if (!planner.buffer_line(dest, scaled_fr_mm_s)) break;
        }
        // LOG_I("neg.x == (dest.x > next_mesh_line_x)\n");
        // LOG_I("neg.x == (dest.x > next_mesh_line_x), istart.x: %d, istart.y: %d, end.x: %f end.y : %f end.e: %f end.i: %f, end.j: %f\n", icell.x, icell.y, dest.x, dest.y, dest.e, dest.i, dest.j);
        // if (dest.i != 0 || dest.i != 0) {
        //   LOG_I("\n\ndest.i: %f, dest.j: %f, normalized_dist: %f start.i: %f, start.j: %f\n", dest.i, dest.j, normalized_dist, start.i, start.j);
        // }
        icell.y += iadd.y;
        cnt.y--;
      }
      else {
        dest.x = next_mesh_line_x;
        if (use_x_dist)
          normalized_dist = (dest.x - start.x) / (end.x - start.x);
        else
          normalized_dist = (dest.y - start.y) / (end.y - start.y);
        // dest.x = CAL_LINE_SEGMENT_END(start.x, end.x, normalized_dist);
        dest.z = CAL_LINE_SEGMENT_END(start.z, end.z, normalized_dist);
        TERN_(LINEAR_AXES >= 4, dest.i = CAL_LINE_SEGMENT_END(start.i, end.i, normalized_dist));
        TERN_(LINEAR_AXES >= 5, dest.j = CAL_LINE_SEGMENT_END(start.j, end.j, normalized_dist));
        TERN_(HAS_EXTRUDERS, dest.e = CAL_LINE_SEGMENT_END(start.e, end.e, normalized_dist));
        // planner.buffer_line(dest, scaled_fr_mm_s);
        if (((dest.x - cur_dest.x)*(dest.x - cur_dest.x) + (dest.y - cur_dest.y) * (dest.y - cur_dest.y)) >= min_valid_segment_len_sqr) {
          // LOG_I("start.x: %f, start.y: %f, end.x: %f, end.y: %f, k: %f, ratio: %f, inf_ratio_flag: %d\n", cur_dest.x, cur_dest.y, dest.x, dest.y, (dest.y - cur_dest.y)/(dest.x - cur_dest.x), ratio, inf_ratio_flag);
          cur_dest = dest;
          // LOG_I("icell.x: %d, icell.y: %d\n", icell.x, icell.y);
          if (!planner.buffer_line(dest, scaled_fr_mm_s)) break;
        }
        // LOG_I("neg.x != (dest.x > next_mesh_line_x)\n");
        // LOG_I("neg.x != (dest.x > next_mesh_line_x), istart.x: %d, istart.y: %d, end.x: %f end.y : %f end.e: %f end.i: %f, end.j: %f\n", icell.x, icell.y, dest.x, dest.y, dest.e, dest.i, dest.j);
        // if (dest.i != 0 || dest.i != 0) {
        //   LOG_I("\n\ndest.i: %f, dest.j: %f, normalized_dist: %f start.i: %f, start.j: %f\n", dest.i, dest.j, normalized_dist, start.i, start.j);
        // }
        icell.x += iadd.x;
        cnt.x--;
      }

      if (cnt.x < 0 || cnt.y < 0) break;
    }

    if (dest != end)
      goto FINAL_MOVE;

    current_position = destination;
    return;
  }
#endif // IS_CARTESIAN && !SEGMENT_LEVELED_MOVES

#endif

#endif // AUTO_BED_LEVELING_BILINEAR
