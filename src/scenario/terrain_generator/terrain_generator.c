#include "terrain_generator.h"

#include "terrain_generator_algorithms.h"

#include "core/random.h"
#include "map/elevation.h"
#include "map/grid.h"
#include "map/property.h"
#include "map/terrain.h"
#include "map/tiles.h"
#include "scenario/editor_map.h"

#include <stdlib.h>
#include <stdint.h>

static int use_fixed_seed = 0;
static unsigned int fixed_seed = 0;

static int terrain_tile_is_passable(int grid_offset)
{
    return !map_terrain_is(grid_offset, TERRAIN_WATER | TERRAIN_ROCK);
}

int terrain_generator_clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

int terrain_generator_random_between(int min_value, int max_value)
{
    return random_between_from_stdlib(min_value, max_value);
}

static void clear_base_terrain(void)
{
    int width = map_grid_width();
    int height = map_grid_height();
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int grid_offset = map_grid_offset(x, y);
            map_terrain_set(grid_offset, TERRAIN_CLEAR);
            map_elevation_set(grid_offset, 0);
        }
    }
}

static void choose_edge_point(int side, int width, int height, int *x, int *y)
{
    switch (side) {
        case 0: // north
            *y = 0;
            *x = terrain_generator_random_between(1, width - 1);
            break;
        case 1: // south
            *y = height - 1;
            *x = terrain_generator_random_between(1, width - 1);
            break;
        case 2: // west
            *x = 0;
            *y = terrain_generator_random_between(1, height - 1);
            break;
        default: // east
            *x = width - 1;
            *y = terrain_generator_random_between(1, height - 1);
            break;
    }

    if (width > 2) {
        *x = terrain_generator_clamp_int(*x, 0, width - 1);
    }
    if (height > 2) {
        *y = terrain_generator_clamp_int(*y, 0, height - 1);
    }
}

int terrain_generator_flood_fill_reachable_land(int start_x, int start_y, uint8_t *reachable_land)
{
    if (!reachable_land) {
        return 0;
    }

    map_grid_clear_u8(reachable_land);

    if (!map_grid_is_inside(start_x, start_y, 1)) {
        return 0;
    }

    int start_offset = map_grid_offset(start_x, start_y);
    if (!terrain_tile_is_passable(start_offset)) {
        return 0;
    }

    const int width = map_grid_width();
    const int height = map_grid_height();
    static const int dx[4] = { 1, -1, 0, 0 };
    static const int dy[4] = { 0, 0, 1, -1 };

    int queue[GRID_SIZE * GRID_SIZE];
    int queue_start = 0;
    int queue_end = 0;
    int reachable_count = 0;

    reachable_land[start_offset] = 1;
    queue[queue_end++] = start_offset;
    reachable_count = 1;

    while (queue_start < queue_end) {
        int current_offset = queue[queue_start++];
        int x = map_grid_offset_to_x(current_offset);
        int y = map_grid_offset_to_y(current_offset);

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }

            int next_offset = map_grid_offset(nx, ny);
            if (reachable_land[next_offset]) {
                continue;
            }
            if (!terrain_tile_is_passable(next_offset)) {
                continue;
            }

            reachable_land[next_offset] = 1;
            queue[queue_end++] = next_offset;
            reachable_count++;
        }
    }

    return reachable_count;
}

static void set_road_tile(int x, int y)
{
    int grid_offset = map_grid_offset(x, y);
    map_terrain_remove(grid_offset,
        TERRAIN_WATER | TERRAIN_TREE | TERRAIN_SHRUB | TERRAIN_ROCK | TERRAIN_MEADOW | TERRAIN_ELEVATION | TERRAIN_ACCESS_RAMP);
    map_terrain_add(grid_offset, TERRAIN_ROAD);
    map_property_clear_constructing(grid_offset);
    map_property_set_multi_tile_size(grid_offset, 1);
    map_property_mark_draw_tile(grid_offset);
    map_tiles_update_area_roads(x, y, 3);
    map_tiles_update_region_empty_land(x - 1, y - 1, x + 1, y + 1);
}

static void add_road_between_points(int start_x, int start_y, int end_x, int end_y)
{
    const uint16_t *segments = terrain_generator_segments();
    int width = map_grid_width();
    int height = map_grid_height();
    int x = start_x;
    int y = start_y;
    int guard = 0;
    int max_steps = width * height * 2;
    int prev_step_x = 0;
    int prev_step_y = 0;
    // Number of initial steps forced inward from the entry edge before normal routing kicks in.
    int min_steps_to_middle = 20;
    int forced_steps = 0;
    int force_dx = 0;
    int force_dy = 0;
    static const int dir_x[4] = { 1, -1, 0, 0 };
    static const int dir_y[4] = { 0, 0, 1, -1 };

    int start_offset = map_grid_offset(start_x, start_y);
    int end_offset = map_grid_offset(end_x, end_y);
    // Connected-component label for walkable land; road must remain inside this component.
    uint16_t road_segment_id = segments[start_offset];

    // Roads are only allowed on passable terrain (not water/rock).
    if (!terrain_tile_is_passable(start_offset) || !terrain_tile_is_passable(end_offset)) {
        return;
    }
    // If endpoints are not in the same reachable land segment, do not attempt a road.
    if (road_segment_id == 0 || segments[end_offset] != road_segment_id) {
        return;
    }

    // Determine inward direction (orthogonal to the starting map edge).
    if (start_y == 0) {
        force_dx = 0;
        force_dy = 1;
    } else if (start_y == height - 1) {
        force_dx = 0;
        force_dy = -1;
    } else if (start_x == 0) {
        force_dx = 1;
        force_dy = 0;
    } else if (start_x == width - 1) {
        force_dx = -1;
        force_dy = 0;
    }

    set_road_tile(x, y);
    while ((x != end_x || y != end_y) && guard++ < max_steps) {
        int current_distance = abs(end_x - x) + abs(end_y - y);
        int apply_forced_direction = (forced_steps < min_steps_to_middle) && (force_dx || force_dy);
        int best_score = 1 << 30;
        int best_x = x;
        int best_y = y;
        int best_step_x = 0;
        int best_step_y = 0;
        int found_move = 0;

        if (apply_forced_direction) {
            int forced_nx = x + force_dx;
            int forced_ny = y + force_dy;
            if (!map_grid_is_inside(forced_nx, forced_ny, 1)) {
                forced_steps = min_steps_to_middle;
                apply_forced_direction = 0;
            } else {
                int forced_offset = map_grid_offset(forced_nx, forced_ny);
                if (!terrain_tile_is_passable(forced_offset) || segments[forced_offset] != road_segment_id) {
                    // Forced inward step is blocked, so stop forcing and continue with regular routing.
                    forced_steps = min_steps_to_middle;
                    apply_forced_direction = 0;
                }
            }
        }

        for (int i = 0; i < 4; i++) {
            int nx = x + dir_x[i];
            int ny = y + dir_y[i];
            if (!map_grid_is_inside(nx, ny, 1)) {
                continue;
            }

            int offset = map_grid_offset(nx, ny);
            if (!terrain_tile_is_passable(offset)) {
                continue;
            }
            if (segments[offset] != road_segment_id) {
                continue;
            }

            if (apply_forced_direction) {
                // During the initial phase, only allow moves along the inward direction.
                if (dir_x[i] != force_dx || dir_y[i] != force_dy) {
                    continue;
                }
            }

            // Manhattan distance to the target; used as the base routing cost.
            int next_distance = abs(end_x - nx) + abs(end_y - ny);
            int step_x = nx - x;
            int step_y = ny - y;
            int score = next_distance * 100;

            if (prev_step_x || prev_step_y) {
                // Heading continuity term: straight is best, turns are penalized, reversal is worst.
                if (step_x == prev_step_x && step_y == prev_step_y) {
                    score -= 25;
                } else if (step_x == -prev_step_x && step_y == -prev_step_y) {
                    score += 80;
                } else {
                    score += 20;
                }
            }

            if (next_distance > current_distance) {
                score += 40;
            }
            score += terrain_generator_random_between(0, 3);

            if (!found_move || score < best_score) {
                found_move = 1;
                best_score = score;
                best_x = nx;
                best_y = ny;
                best_step_x = step_x;
                best_step_y = step_y;
            }
        }

        if (!found_move) {
            break;
        }

        x = best_x;
        y = best_y;
        prev_step_x = best_step_x;
        prev_step_y = best_step_y;
        forced_steps++;
        set_road_tile(x, y);
    }
}

static void set_entry_exit_points(void)
{
    int width = map_grid_width();
    int height = map_grid_height();

    const uint16_t *segments = terrain_generator_segments();

    int is_passable = 0;
    int entry_side = 0;
    int entry_x = 0;
    int entry_y = 0;
    int exit_x = 0;
    int exit_y = 0;
    while (!is_passable) {
        entry_side = terrain_generator_random_between(0, 4);
        choose_edge_point(entry_side, width, height, &entry_x, &entry_y);
        is_passable = terrain_tile_is_passable(map_grid_offset(entry_x, entry_y));
    }

    int entry_offset = map_grid_offset(entry_x, entry_y);
    uint16_t entry_segment_id = segments[entry_offset];
    int found_exit = 0;
    int candidate_count = 0;

    if (entry_segment_id > 0) {
        for (int ny = 0; ny < height; ny++) {
            for (int nx = 0; nx < width; nx++) {
                if (nx != 0 && nx != width - 1 && ny != 0 && ny != height - 1) {
                    continue;
                }

                if (nx == entry_x && ny == entry_y) {
                    continue;
                }

                int offset = map_grid_offset(nx, ny);
                if (!terrain_tile_is_passable(offset)) {
                    continue;
                }
                if (segments[offset] != entry_segment_id) {
                    continue;
                }

                candidate_count++;
                if (terrain_generator_random_between(0, candidate_count) == 0) {
                    exit_x = nx;
                    exit_y = ny;
                    found_exit = 1;
                }
            }
        }
    }

    if (!found_exit) {
        exit_x = entry_x;
        exit_y = entry_y;
    }

    scenario_editor_set_entry_point(entry_x, entry_y);
    scenario_editor_set_exit_point(exit_x, exit_y);

    add_road_between_points(entry_x, entry_y, exit_x, exit_y);
}

void terrain_generator_generate(terrain_generator_algorithm algorithm)
{
    if (use_fixed_seed) {
        random_set_stdlib_seed(fixed_seed);
    } else {
        fixed_seed = terrain_generator_random_between(1, 0x7fffffff);
        use_fixed_seed = 1;
        random_set_stdlib_seed(fixed_seed);
    }

    clear_base_terrain();

    switch (algorithm) {
        case TERRAIN_GENERATOR_RIVER:
            terrain_generator_river_map(fixed_seed);
            break;
        case TERRAIN_GENERATOR_RANDOM:
        default:
            terrain_generator_random_terrain();
            break;
    }

    set_entry_exit_points();
    random_clear_stdlib_seed();
}

void terrain_generator_set_seed(int enabled, unsigned int seed)
{
    use_fixed_seed = enabled != 0;
    fixed_seed = seed;
}
