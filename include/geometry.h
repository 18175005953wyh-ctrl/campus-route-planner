#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "graph.h"

double euclidean_distance(const Location *a, const Location *b);
double graph_heuristic_scale(const Graph *graph);
/* Build once per query, never in the expansion loop. Returns effective scale.
   A conservative rounding margin and edge check protect double arithmetic. */
double build_goal_heuristic(const Graph *graph, int goal, double values[MAX_LOCATIONS]);

#endif
