#include "geometry.h"
#include <math.h>
#include <stddef.h>

double euclidean_distance(const Location *a, const Location *b)
{
    double dx = (double)a->x - (double)b->x;
    double dy = (double)a->y - (double)b->y;
    return hypot(dx, dy);
}

double graph_heuristic_scale(const Graph *graph)
{
    double scale = 0.0;
    int i, j, have_edge = 0;
    if (!graph_has_complete_coordinates(graph)) return 0.0;
    for (i = 0; i < graph->count; ++i) {
        for (j = 0; j < graph->count; ++j) {
            int weight = graph->adjacency[i][j];
            double length, ratio;
            if (i == j || weight < 0 || weight >= INF_DISTANCE) continue;
            length = euclidean_distance(&graph->locations[i], &graph->locations[j]);
            if (length == 0.0) continue;
            ratio = (double)weight / length;
            if (!have_edge || ratio < scale) scale = ratio;
            have_edge = 1;
        }
    }
    return scale;
}

double build_goal_heuristic(const Graph *graph, int goal, double values[MAX_LOCATIONS])
{
    double scale;
    int i, j;
    for (i = 0; i < MAX_LOCATIONS; ++i) values[i] = 0.0;
    if (!graph_has_complete_coordinates(graph) || goal < 0 || goal >= graph->count) return 0.0;
    scale = graph_heuristic_scale(graph) * (1.0 - 1e-12);
    for (i = 0; i < graph->count; ++i)
        values[i] = scale * euclidean_distance(&graph->locations[i], &graph->locations[goal]);
    for (i = 0; i < graph->count; ++i) {
        for (j = 0; j < graph->count; ++j) {
            int weight = graph->adjacency[i][j];
            if (weight >= 0 && weight < INF_DISTANCE &&
                values[i] > (double)weight + values[j]) {
                /* Extreme rounding must weaken guidance, never correctness. */
                for (i = 0; i < MAX_LOCATIONS; ++i) values[i] = 0.0;
                return 0.0;
            }
        }
    }
    return scale;
}
