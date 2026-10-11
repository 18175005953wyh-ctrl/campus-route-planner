#include "graph.h"
#include "input.h"
#include "pathfinder.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PATH_CAPACITY 1024
typedef enum { ALGORITHM_DIJKSTRA, ALGORITHM_ASTAR, ALGORITHM_COMPARE } Algorithm;

static void usage(FILE *stream)
{
    fputs("Usage: campus_route_planner [--data DIRECTORY] [--algorithm dijkstra|astar|compare]\n"
          "Default: dijkstra; data in current directory, then data beside the executable.\n"
          "A* and compare require complete coordinates.csv. Use --help alone.\n", stream);
}

static int join_path(char *output, const char *directory, const char *file)
{
    int written = snprintf(output, PATH_CAPACITY, "%s/%s", directory, file);
    return written >= 0 && written < PATH_CAPACITY;
}

static int parse_options(int argc, char *argv[], char *directory, Algorithm *algorithm)
{
    FILE *probe;
    const char *slash, *backslash;
    size_t prefix;
    int i, have_data = 0, have_algorithm = 0;
    *algorithm = ALGORITHM_DIJKSTRA;
    if (argc == 2 && strcmp(argv[1], "--help") == 0) {
        usage(stdout);
        return 0;
    }
    for (i = 1; i < argc; ++i) {
        if (i + 1 >= argc || argv[i + 1][0] == '\0' || strncmp(argv[i + 1], "--", 2) == 0) goto invalid;
        if (strcmp(argv[i], "--data") == 0) {
            if (have_data || strlen(argv[i + 1]) >= PATH_CAPACITY) goto invalid;
            ++i;
            memcpy(directory, argv[i], strlen(argv[i]) + 1);
            have_data = 1;
        } else if (strcmp(argv[i], "--algorithm") == 0) {
            if (have_algorithm) goto invalid;
            ++i;
            if (strcmp(argv[i], "dijkstra") == 0) *algorithm = ALGORITHM_DIJKSTRA;
            else if (strcmp(argv[i], "astar") == 0) *algorithm = ALGORITHM_ASTAR;
            else if (strcmp(argv[i], "compare") == 0) *algorithm = ALGORITHM_COMPARE;
            else goto invalid;
            have_algorithm = 1;
        } else {
            goto invalid;
        }
    }
    if (have_data) return 1;
    memcpy(directory, "data", 5);
    probe = fopen("data/locations.csv", "r");
    if (probe != NULL) { fclose(probe); return 1; }
    slash = strrchr(argv[0], '/');
    backslash = strrchr(argv[0], '\\');
    if (backslash != NULL && (slash == NULL || backslash > slash)) slash = backslash;
    if (slash != NULL) {
        prefix = (size_t)(slash - argv[0]) + 1;
        if (prefix + 5 > PATH_CAPACITY) return -1;
        memcpy(directory, argv[0], prefix);
        memcpy(directory + prefix, "data", 5);
    }
    return 1;
invalid:
    fputs("Invalid arguments: expected unique --data and --algorithm with valid values.\n", stderr);
    usage(stderr);
    return -1;
}

static int read_location(const Graph *graph, const char *prompt, int *index)
{
    int id;
    for (;;) {
        if (!read_integer(prompt, 1, INT_MAX, &id)) return 0;
        *index = find_location_index_by_id(graph, id);
        if (*index >= 0) return 1;
        puts("Unknown location ID. Please try again.");
    }
}

static void print_metrics(const char *label, const PathResult *result)
{
    printf("%s: ", label);
    if (result->found) printf("distance=%d m", result->distance);
    else printf("unreachable");
    printf(", expanded=%d, relaxed=%d\n", result->expanded_nodes, result->relaxed_edges);
}

static int find_route(const Graph *graph, Algorithm algorithm)
{
    int start, end;
    PathResult result, reference;
    if (graph->count == 0) { puts("No locations loaded."); return 1; }
    if (!read_location(graph, "Start location ID: ", &start) ||
        !read_location(graph, "Destination location ID: ", &end)) return 1;
    if (start == end) puts("Start and destination are the same location.");
    if (algorithm == ALGORITHM_DIJKSTRA) {
        result = dijkstra_shortest_path(graph, start, end);
        print_path(graph, &result);
        return 1;
    }
    result = astar_shortest_path(graph, start, end);
    if (algorithm == ALGORITHM_COMPARE) {
        reference = dijkstra_shortest_path(graph, start, end);
        if (!path_results_agree(graph, &reference, &result, start, end)) {
            fputs("Internal consistency error: route results disagree or contain an invalid path.\n", stderr);
            return 0;
        }
        puts("\nAlgorithm comparison\nDijkstra route:");
        print_path(graph, &reference);
        print_metrics("Dijkstra", &reference);
    }
    puts("A* route:");
    print_path(graph, &result);
    print_metrics("A*", &result);
    printf("Heuristic scale: %.12g\n", result.heuristic_scale);
    if (result.heuristic_scale == 0.0) puts("Zero heuristic: A* degenerates to Dijkstra.");
    if (algorithm == ALGORITHM_COMPARE) puts("Distance agreement: yes\nPath validity: yes");
    return 1;
}

int main(int argc, char *argv[])
{
    Graph graph;
    char directory[PATH_CAPACITY], locations[PATH_CAPACITY], roads[PATH_CAPACITY];
    char coordinates[PATH_CAPACITY];
    char keyword[NAME_LENGTH];
    Algorithm algorithm;
    int option, status = parse_options(argc, argv, directory, &algorithm);
    if (status == 0) return EXIT_SUCCESS;
    if (status < 0 || !join_path(locations, directory, "locations.csv") || !join_path(roads, directory, "roads.csv")) {
        fputs("Data directory arguments are invalid or too long.\n", stderr);
        return EXIT_FAILURE;
    }
    graph_init(&graph);
    if (!load_locations(&graph, locations) || !load_roads(&graph, roads)) {
        fputs("Map load failed. Check files or use --data DIRECTORY.\n", stderr);
        return EXIT_FAILURE;
    }
    if (algorithm != ALGORITHM_DIJKSTRA &&
        (!join_path(coordinates, directory, "coordinates.csv") || !load_coordinates(&graph, coordinates) ||
         !graph_has_complete_coordinates(&graph))) {
        fputs("A* and compare require a readable, complete coordinates.csv for every location.\n", stderr);
        return EXIT_FAILURE;
    }
    printf("Loaded %d locations.\n", graph.count);
    for (;;) {
        puts("\n===== Campus Route Planner =====\n1. List all locations\n2. Show direct roads\n"
             "3. Find shortest route\n4. Search location\n5. Show map summary\n0. Exit");
        if (!read_integer("Select an option: ", 0, 5, &option) || option == 0) break;
        switch (option) {
        case 1: list_locations(&graph); break;
        case 2: list_roads(&graph); break;
        case 3:
            if (!find_route(&graph, algorithm)) return EXIT_FAILURE;
            break;
        case 4:
            if (read_keyword("Name keyword: ", keyword, (int)sizeof keyword)) search_locations(&graph, keyword);
            break;
        case 5: show_map_summary(&graph); break;
        default: break;
        }
    }
    puts("Goodbye.");
    return ferror(stdin) ? EXIT_FAILURE : EXIT_SUCCESS;
}
