#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "graph.h"

typedef struct {
    int found;
    int distance;
    int path[MAX_LOCATIONS];
    int path_length;
    int expanded_nodes;
    int relaxed_edges;
    double heuristic_scale;
} PathResult;

PathResult dijkstra_shortest_path(const Graph *graph, int start_index, int end_index);
/* Missing coordinates return found=0; the CLI rejects them before any query. */
PathResult astar_shortest_path(const Graph *graph, int start_index, int end_index);
int path_result_is_valid(const Graph *graph, const PathResult *result, int start, int end);
int path_results_agree(const Graph *graph, const PathResult *a, const PathResult *b, int start, int end);
void print_path(const Graph *graph, const PathResult *result);

#endif
