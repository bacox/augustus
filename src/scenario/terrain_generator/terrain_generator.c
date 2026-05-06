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

static void adjust_point_to_land(int *x, int *y, int width, int height)
{
    int grid_offset = map_grid_offset(*x, *y);
    if (terrain_tile_is_passable(grid_offset)) {
        return;
    }

    for (int radius = 1; radius <= 10; radius++) {
        for (int dy = -radius; dy <= radius; dy++) {
            for (int dx = -radius; dx <= radius; dx++) {
                int nx = *x + dx;
                int ny = *y + dy;
                if (!map_grid_is_inside(nx, ny, 1)) {
                    continue;
                }
                int offset = map_grid_offset(nx, ny);
                if (terrain_tile_is_passable(offset)) {
                    *x = nx;
                    *y = ny;
                    return;
                }
            }
        }
    }
}

static int choose_reachable_edge_point(int side, int width, int height, const uint8_t *reachable_land, int *x, int *y)
{
    int candidate_x[GRID_SIZE];
    int candidate_y[GRID_SIZE];
    int candidate_count = 0;

    if (side == 0 || side == 1) {
        int edge_y = (side == 0) ? 0 : height - 1;
        for (int edge_x = 0; edge_x < width; edge_x++) {
            int offset = map_grid_offset(edge_x, edge_y);
            if (!reachable_land[offset] || !terrain_tile_is_passable(offset)) {
                continue;
            }
            if (candidate_count >= GRID_SIZE) {
                break;
            }
            candidate_x[candidate_count] = edge_x;
            candidate_y[candidate_count] = edge_y;
            candidate_count++;
        }
    } else {
        int edge_x = (side == 2) ? 0 : width - 1;
        for (int edge_y = 0; edge_y < height; edge_y++) {
            int offset = map_grid_offset(edge_x, edge_y);
            if (!reachable_land[offset] || !terrain_tile_is_passable(offset)) {
                continue;
            }
            if (candidate_count >= GRID_SIZE) {
                break;
            }
            candidate_x[candidate_count] = edge_x;
            candidate_y[candidate_count] = edge_y;
            candidate_count++;
        }
    }

    if (candidate_count <= 0) {
        return 0;
    }

    int choice = terrain_generator_random_between(0, candidate_count);
    *x = candidate_x[choice];
    *y = candidate_y[choice];
    return 1;
}

static int choose_nearest_reachable_point(int start_x, int start_y, int width, int height, const uint8_t *reachable_land, int *x, int *y)
{
    int found = 0;
    int best_distance = 0;
    int best_x = 0;
    int best_y = 0;

    for (int ny = 0; ny < height; ny++) {
        for (int nx = 0; nx < width; nx++) {
            int offset = map_grid_offset(nx, ny);
            if (!reachable_land[offset] || !terrain_tile_is_passable(offset)) {
                continue;
            }

            int distance = abs(nx - start_x) + abs(ny - start_y);
            if (!found || distance < best_distance) {
                found = 1;
                best_distance = distance;
                best_x = nx;
                best_y = ny;
            }
        }
    }

    if (!found) {
        return 0;
    }

    *x = best_x;
    *y = best_y;
    return 1;
}

static int choose_farthest_reachable_edge_point(int entry_x, int entry_y, int width, int height, const uint8_t *reachable_land, int *x, int *y)
{
    int found = 0;
    int best_distance = -1;
    int best_x = entry_x;
    int best_y = entry_y;
    int tie_count = 0;

    for (int ny = 0; ny < height; ny++) {
        for (int nx = 0; nx < width; nx++) {
            if (nx != 0 && nx != width - 1 && ny != 0 && ny != height - 1) {
                continue;
            }

            int offset = map_grid_offset(nx, ny);
            if (!reachable_land[offset] || !terrain_tile_is_passable(offset)) {
                continue;
            }

            int distance = abs(nx - entry_x) + abs(ny - entry_y);
            if (distance > best_distance) {
                found = 1;
                best_distance = distance;
                best_x = nx;
                best_y = ny;
                tie_count = 1;
            } else if (distance == best_distance) {
                tie_count++;
                if (terrain_generator_random_between(0, tie_count) == 0) {
                    best_x = nx;
                    best_y = ny;
                }
            }
        }
    }

    if (!found) {
        return 0;
    }

    *x = best_x;
    *y = best_y;
    return 1;
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
    int x = start_x;
    int y = start_y;
    int guard = 0;
    int max_steps = map_grid_width() * map_grid_height() * 2;

    set_road_tile(x, y);
    while ((x != end_x || y != end_y) && guard++ < max_steps) {
        int dx = end_x - x;
        int dy = end_y - y;
        int step_x = (dx > 0) - (dx < 0);
        int step_y = (dy > 0) - (dy < 0);
        int abs_dx = abs(dx);
        int abs_dy = abs(dy);
        int roll = terrain_generator_random_between(0, 100);

        if (abs_dx >= abs_dy) {
            if (step_x && roll < 70) {
                x += step_x;
            } else if (step_y) {
                y += step_y;
            }
        } else {
            if (step_y && roll < 70) {
                y += step_y;
            } else if (step_x) {
                x += step_x;
            }
        }

        x = terrain_generator_clamp_int(x, 0, map_grid_width() - 1);
        y = terrain_generator_clamp_int(y, 0, map_grid_height() - 1);
        set_road_tile(x, y);
    }
}

static void set_entry_exit_points(void)
{
    int width = map_grid_width();
    int height = map_grid_height();

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

    uint8_t reachable_land[GRID_SIZE * GRID_SIZE];
    int reachable_count = terrain_generator_flood_fill_reachable_land(entry_x, entry_y, reachable_land);

    // Find a tile on the edge that is reachable.
    if (reachable_count > 0) {
        if (!choose_farthest_reachable_edge_point(entry_x, entry_y, width, height, reachable_land, &exit_x, &exit_y)) {
            if (!choose_nearest_reachable_point(entry_x, entry_y, width, height, reachable_land, &exit_x, &exit_y)) {
                exit_x = entry_x;
                exit_y = entry_y;
            }
        }
    } else {
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
