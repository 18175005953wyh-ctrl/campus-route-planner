#include "pathfinder.h"
#include "geometry.h"
#include <stdio.h>

static void reconstruct_path(PathResult *result, const int previous[], int start, int end, int distance)
{
    int reverse[MAX_LOCATIONS], length = 0, current = end, i;
    while (current >= 0 && current < MAX_LOCATIONS && length < MAX_LOCATIONS) {
        reverse[length++] = current;
        if (current == start) break;
        current = previous[current];
    }
    if (length == 0 || reverse[length - 1] != start) return;
    result->found = 1;
    result->distance = distance;
    result->path_length = length;
    for (i = 0; i < length; ++i) result->path[i] = reverse[length - 1 - i];
}

static PathResult shortest_path(const Graph *graph, int start_index, int end_index, int use_astar)
{
    PathResult result = {0};
    int distance[MAX_LOCATIONS], visited[MAX_LOCATIONS] = {0}, previous[MAX_LOCATIONS];
    double heuristic[MAX_LOCATIONS] = {0};
    int step, i, current, neighbor, weight, candidate;
    result.distance = INF_DISTANCE;
    if (graph == NULL || graph->count < 0 || graph->count > MAX_LOCATIONS) return result;
    if (start_index < 0 || end_index < 0 || start_index >= graph->count || end_index >= graph->count) return result;
    if (use_astar) {
        if (!graph_has_complete_coordinates(graph)) return result;
        result.heuristic_scale = build_goal_heuristic(graph, end_index, heuristic);
    }
    for (i = 0; i < graph->count; ++i) { distance[i] = INF_DISTANCE; previous[i] = -1; }
    distance[start_index] = 0;
    for (step = 0; step < graph->count; ++step) {
        current = -1;
        for (i = 0; i < graph->count; ++i)
            if (!visited[i] && distance[i] < INF_DISTANCE &&
                (current == -1 || (double)distance[i] + heuristic[i] <
                                 (double)distance[current] + heuristic[current])) current = i;
        if (current == -1) break;
        visited[current] = 1;
        ++result.expanded_nodes;
        if (current == end_index) break;
        for (neighbor = 0; neighbor < graph->count; ++neighbor) {
            weight = graph->adjacency[current][neighbor];
            if (visited[neighbor] || weight < 0 || weight >= INF_DISTANCE) continue;
            /* Check before addition, so even a malformed graph cannot overflow. */
            if (distance[current] > INF_DISTANCE - weight) continue;
            candidate = distance[current] + weight;
            if (candidate < distance[neighbor]) {
                distance[neighbor] = candidate;
                previous[neighbor] = current;
                ++result.relaxed_edges;
            }
        }
    }
    if (distance[end_index] == INF_DISTANCE) return result;
    reconstruct_path(&result, previous, start_index, end_index, distance[end_index]);
    return result;
}

PathResult dijkstra_shortest_path(const Graph *graph, int start_index, int end_index)
{
    return shortest_path(graph, start_index, end_index, 0);
}

PathResult astar_shortest_path(const Graph *graph, int start_index, int end_index)
{
    return shortest_path(graph, start_index, end_index, 1);
}

int path_result_is_valid(const Graph *graph, const PathResult *result, int start, int end)
{
    int i, j, sum = 0;
    if (graph == NULL || result == NULL || graph->count < 1 || graph->count > MAX_LOCATIONS ||
        start < 0 || end < 0 || start >= graph->count || end >= graph->count || !result->found ||
        result->path_length < 1 || result->path_length > graph->count ||
        result->path[0] != start || result->path[result->path_length - 1] != end) return 0;
    for (i = 0; i < result->path_length; ++i) {
        int node = result->path[i];
        if (node < 0 || node >= graph->count) return 0;
        for (j = 0; j < i; ++j) if (result->path[j] == node) return 0;
        if (i > 0) {
            int weight = graph->adjacency[result->path[i - 1]][node];
            if (weight <= 0 || weight >= INF_DISTANCE || sum > INF_DISTANCE - weight) return 0;
            sum += weight;
        }
    }
    return sum == result->distance;
}

int path_results_agree(const Graph *graph, const PathResult *a, const PathResult *b, int start, int end)
{
    if (graph == NULL || a == NULL || b == NULL || graph->count < 1 || graph->count > MAX_LOCATIONS ||
        start < 0 || end < 0 || start >= graph->count || end >= graph->count ||
        a->found != b->found || a->distance != b->distance) return 0;
    if (!a->found) return a->distance == INF_DISTANCE && a->path_length == 0 && b->path_length == 0;
    return path_result_is_valid(graph, a, start, end) && path_result_is_valid(graph, b, start, end);
}

void print_path(const Graph *graph, const PathResult *result)
{
    int i;
    if (!result->found) { puts("No route available between these locations."); return; }
    puts("Shortest route:");
    for (i = 0; i < result->path_length; ++i)
        printf("%s%s", i ? " -> " : "", graph->locations[result->path[i]].name);
    printf("\n\nTotal distance: %d m\n", result->distance);
}
