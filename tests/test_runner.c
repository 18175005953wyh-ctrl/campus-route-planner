#include "graph.h"
#include "geometry.h"
#include "input.h"
#include "pathfinder.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *fixtures = "tests/fixtures";
static int passed = 0, failed = 0;

#define REQUIRE(condition) do { if (!(condition)) { \
    printf("  Assertion failed at line %d: %s\n", __LINE__, #condition); return 0; } } while (0)

static int load_fixture(Graph *graph, const char *roads)
{
    char path[1024];
    graph_init(graph);
    if (snprintf(path, sizeof path, "%s/test_locations.csv", fixtures) >= (int)sizeof path) return 0;
    if (!load_locations(graph, path)) return 0;
    if (roads == NULL) return 1;
    if (snprintf(path, sizeof path, "%s/%s", fixtures, roads) >= (int)sizeof path) return 0;
    return load_roads(graph, path);
}

static int write_fixture(const char *content)
{
    FILE *file = fopen("route-test-generated.csv", "w");
    int ok;
    if (file == NULL) return 0;
    ok = fputs(content, file) >= 0;
    if (fclose(file) != 0) ok = 0;
    return ok;
}

static int valid_path(const Graph *graph, PathResult result, int start, int end)
{
    int i, j, sum = 0;
    if (!result.found || result.path_length < 1 || result.path_length > graph->count ||
        result.path[0] != start || result.path[result.path_length - 1] != end) return 0;
    for (i = 0; i < result.path_length; ++i) {
        if (result.path[i] < 0 || result.path[i] >= graph->count) return 0;
        for (j = 0; j < i; ++j) if (result.path[i] == result.path[j]) return 0;
        if (i > 0) {
            int weight = graph->adjacency[result.path[i - 1]][result.path[i]];
            if (weight == INF_DISTANCE) return 0;
            sum += weight;
        }
    }
    return sum == result.distance;
}

static int test_initialization(void)
{
    Graph graph;
    graph_init(&graph);
    REQUIRE(graph.count == 0);
    REQUIRE(graph.adjacency[0][0] == 0);
    REQUIRE(graph.adjacency[0][49] == INF_DISTANCE);
    return 1;
}

static int test_locations(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(graph.count == 5);
    REQUIRE(strcmp(graph.locations[1].name, "Library") == 0);
    return 1;
}

static int test_roads(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    REQUIRE(graph.adjacency[0][1] == 5 && graph.adjacency[1][0] == 5);
    REQUIRE(graph.adjacency[0][0] == 0 && graph.adjacency[0][4] == INF_DISTANCE);
    return 1;
}

static int test_id_mapping(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(find_location_index_by_id(&graph, 30) == 2);
    REQUIRE(find_location_index_by_id(&graph, 3) == -1);
    return 1;
}

static int test_direct_route(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    result = dijkstra_shortest_path(&graph, 0, 1);
    REQUIRE(result.distance == 5 && result.path_length == 2);
    REQUIRE(valid_path(&graph, result, 0, 1));
    return 1;
}

static int test_multi_step(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    result = dijkstra_shortest_path(&graph, 0, 3);
    REQUIRE(result.distance == 15 && result.path_length == 4);
    REQUIRE(result.path[1] == 1 && result.path[2] == 2);
    REQUIRE(valid_path(&graph, result, 0, 3));
    return 1;
}

static int test_same_location(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    result = dijkstra_shortest_path(&graph, 4, 4);
    REQUIRE(result.found && result.distance == 0 && result.path_length == 1 && result.path[0] == 4);
    return 1;
}

static int test_disconnected(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_fixture(&graph, "disconnected_roads.csv"));
    result = dijkstra_shortest_path(&graph, 0, 3);
    REQUIRE(!result.found && result.distance == INF_DISTANCE && result.path_length == 0);
    return 1;
}

static int test_isolated(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    REQUIRE(!dijkstra_shortest_path(&graph, 0, 4).found);
    REQUIRE(!dijkstra_shortest_path(&graph, 4, 0).found);
    return 1;
}

static int test_invalid_indices(void)
{
    Graph graph;
    graph_init(&graph);
    REQUIRE(!dijkstra_shortest_path(&graph, 0, 0).found);
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(!dijkstra_shortest_path(&graph, -1, 2).found);
    REQUIRE(!dijkstra_shortest_path(&graph, 0, graph.count).found);
    return 1;
}

static int test_search(void)
{
    Graph graph;
    int matches[MAX_LOCATIONS];
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(find_location_matches(&graph, "lIbRa", matches) == 1 && matches[0] == 1);
    REQUIRE(find_location_matches(&graph, "a", matches) == 5);
    REQUIRE(find_location_matches(&graph, "missing", matches) == 0);
    return 1;
}

static int test_empty_search(void)
{
    Graph graph;
    int matches[MAX_LOCATIONS];
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(find_location_matches(&graph, "", matches) == 0);
    REQUIRE(find_location_matches(&graph, " \t ", matches) == 0);
    return 1;
}

static int test_invalid_road_api(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(!graph_add_road(&graph, 10, 20, -5));
    REQUIRE(!graph_add_road(&graph, 10, 20, 0));
    REQUIRE(!graph_add_road(&graph, 10, 10, 5));
    REQUIRE(!graph_add_road(&graph, 10, 999, 5));
    REQUIRE(!graph_add_road(&graph, 10, 20, MAX_ROAD_DISTANCE + 1));
    REQUIRE(graph.adjacency[0][1] == INF_DISTANCE);
    REQUIRE(graph.adjacency[0][0] == 0);
    return 1;
}

static int test_invalid_road_csv(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("10,20,-1\n10,20,0\n10,99,3\n10,10,4\nwrong\n10,20,9,extra\n10,20,999999999999999\n\n20,30,8\n"));
    REQUIRE(load_roads(&graph, "route-test-generated.csv"));
    REQUIRE(graph.adjacency[0][1] == INF_DISTANCE && graph.adjacency[1][2] == 8);
    REQUIRE(calculate_map_summary(&graph).road_count == 1);
    return 1;
}

static int test_invalid_location_csv(void)
{
    Graph graph;
    graph_init(&graph);
    REQUIRE(write_fixture("\n10,Valid,Study\n10,Duplicate,Study\n0,Zero,Study\nabc,Wrong,Study\n20,,Study\n20,X,Y,Z\n30,Next,Education\n"));
    REQUIRE(load_locations(&graph, "route-test-generated.csv"));
    REQUIRE(graph.count == 2 && graph.locations[1].id == 30);
    return 1;
}

static int test_duplicate_roads(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("10,20,12\n20,10,7\n10,20,9\n"));
    REQUIRE(load_roads(&graph, "route-test-generated.csv"));
    REQUIRE(graph.adjacency[0][1] == 7 && graph.adjacency[1][0] == 7);
    REQUIRE(calculate_map_summary(&graph).road_count == 1);
    return 1;
}

static int test_summary(void)
{
    Graph graph;
    MapSummary summary;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    summary = calculate_map_summary(&graph);
    REQUIRE(summary.road_count == 5 && summary.total_distance == 85);
    REQUIRE(summary.isolated_count == 1 && summary.max_degree == 3);
    REQUIRE(summary.degrees[1] == 3 && summary.degrees[3] == 3);
    return 1;
}

static int test_empty_summary(void)
{
    Graph graph;
    MapSummary summary;
    graph_init(&graph);
    summary = calculate_map_summary(&graph);
    REQUIRE(summary.road_count == 0 && summary.total_distance == 0 && summary.max_degree == 0);
    REQUIRE(summary.isolated_count == 0);
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(calculate_map_summary(&graph).isolated_count == 5);
    return 1;
}

static int test_location_capacity(void)
{
    Graph graph;
    int i;
    graph_init(&graph);
    for (i = 1; i <= MAX_LOCATIONS; ++i) REQUIRE(graph_add_location(&graph, i, "Place", "Type"));
    REQUIRE(!graph_add_location(&graph, 51, "Overflow", "Type"));
    REQUIRE(!graph_add_location(&graph, 1, "Duplicate", "Type"));
    REQUIRE(graph.count == MAX_LOCATIONS);
    return 1;
}

static int test_csv_capacity(void)
{
    Graph graph;
    FILE *file = fopen("route-test-generated.csv", "w");
    int i;
    REQUIRE(file != NULL);
    for (i = 1; i <= 51; ++i) fprintf(file, "%d,Place,Type\n", i);
    REQUIRE(fclose(file) == 0);
    graph_init(&graph);
    REQUIRE(load_locations(&graph, "route-test-generated.csv"));
    REQUIRE(graph.count == 50 && find_location_index_by_id(&graph, 51) == -1);
    return 1;
}

static int test_max_distance_chain(void)
{
    Graph graph;
    PathResult result;
    int i;
    graph_init(&graph);
    for (i = 1; i <= MAX_LOCATIONS; ++i) REQUIRE(graph_add_location(&graph, i, "Place", "Type"));
    for (i = 1; i < MAX_LOCATIONS; ++i) REQUIRE(graph_add_road(&graph, i, i + 1, MAX_ROAD_DISTANCE));
    result = dijkstra_shortest_path(&graph, 0, MAX_LOCATIONS - 1);
    REQUIRE(result.distance == MAX_ROAD_DISTANCE * (MAX_LOCATIONS - 1));
    REQUIRE(result.path_length == MAX_LOCATIONS && valid_path(&graph, result, 0, MAX_LOCATIONS - 1));
    return 1;
}

static int test_large_summary(void)
{
    Graph graph;
    int i, j;
    MapSummary summary;
    graph_init(&graph);
    for (i = 1; i <= MAX_LOCATIONS; ++i) REQUIRE(graph_add_location(&graph, i, "Place", "Type"));
    for (i = 1; i <= MAX_LOCATIONS; ++i)
        for (j = i + 1; j <= MAX_LOCATIONS; ++j) REQUIRE(graph_add_road(&graph, i, j, MAX_ROAD_DISTANCE));
    summary = calculate_map_summary(&graph);
    REQUIRE(summary.road_count == 1225);
    REQUIRE(summary.total_distance == 1225LL * MAX_ROAD_DISTANCE && summary.total_distance > INT_MAX);
    return 1;
}

static int test_equal_routes(void)
{
    Graph graph;
    PathResult first, second;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(graph_add_road(&graph, 10, 20, 5));
    REQUIRE(graph_add_road(&graph, 10, 30, 5));
    REQUIRE(graph_add_road(&graph, 20, 40, 5));
    REQUIRE(graph_add_road(&graph, 30, 40, 5));
    first = dijkstra_shortest_path(&graph, 0, 3);
    second = dijkstra_shortest_path(&graph, 0, 3);
    REQUIRE(first.distance == 10 && first.path[1] == 1);
    REQUIRE(first.path[1] == second.path[1] && valid_path(&graph, first, 0, 3));
    return 1;
}

static int test_integer_validation(void)
{
    int value;
    REQUIRE(parse_integer(" 42 \t", 1, 100, &value) && value == 42);
    REQUIRE(!parse_integer("", 1, 100, &value));
    REQUIRE(!parse_integer("4x", 1, 100, &value));
    REQUIRE(!parse_integer("0", 1, 100, &value));
    REQUIRE(!parse_integer("999999999999999999999999", 1, INT_MAX, &value));
    return 1;
}

static int test_long_line_recovery(void)
{
    char line[8];
    FILE *file;
    REQUIRE(write_fixture("01234567890123456789\n42\n"));
    file = fopen("route-test-generated.csv", "r");
    REQUIRE(file != NULL);
    REQUIRE(read_line(file, line, (int)sizeof line) == -1);
    REQUIRE(read_line(file, line, (int)sizeof line) == 1 && strcmp(line, "42") == 0);
    REQUIRE(read_line(file, line, (int)sizeof line) == 0);
    REQUIRE(fclose(file) == 0);
    return 1;
}

static int test_bom_and_whitespace(void)
{
    Graph graph;
    graph_init(&graph);
    REQUIRE(write_fixture("\xEF\xBB\xBF" " 10 , Main Gate , Entrance \n \t\n20,Library,Study"));
    REQUIRE(load_locations(&graph, "route-test-generated.csv"));
    REQUIRE(graph.count == 2 && strcmp(graph.locations[0].name, "Main Gate") == 0);
    return 1;
}

static int test_bad_labels(void)
{
    Graph graph;
    char text[NAME_LENGTH + 1];
    graph_init(&graph);
    memset(text, 'a', sizeof text - 1); text[sizeof text - 1] = '\0';
    REQUIRE(!graph_add_location(&graph, 1, text, "Type"));
    REQUIRE(!graph_add_location(&graph, 1, " ", "Type"));
    REQUIRE(!graph_add_location(&graph, 1, "A,B", "Type"));
    REQUIRE(!graph_add_location(&graph, 1, "Name", "T\t"));
    text[NAME_LENGTH - 1] = '\0';
    REQUIRE(graph_add_location(&graph, 1, text, "Type"));
    return 1;
}

static int test_missing_files(void)
{
    Graph graph;
    graph_init(&graph);
    REQUIRE(!load_locations(&graph, "route-test-missing-dir/locations.csv"));
    REQUIRE(!load_roads(&graph, "route-test-missing-dir/roads.csv"));
    return 1;
}

static int test_reference_all_pairs(void)
{
    Graph graph;
    int reference[8][8], i, j, k;
    PathResult result;
    graph_init(&graph);
    for (i = 0; i < 8; ++i) REQUIRE(graph_add_location(&graph, i + 10, "Place", "Type"));
    for (i = 0; i < 8; ++i) {
        graph.locations[i].x = i * 31;
        graph.locations[i].y = (i % 3) * -47;
        graph.locations[i].has_coordinates = 1;
    }
    for (i = 0; i < 7; ++i)
        for (j = i + 1; j < 7; ++j)
            if ((i + j) % 3 != 0) REQUIRE(graph_add_road(&graph, i + 10, j + 10, (i * 17 + j * 11) % 40 + 1));
    for (i = 0; i < 8; ++i) for (j = 0; j < 8; ++j) reference[i][j] = graph.adjacency[i][j];
    /* Independent Floyd-Warshall reference checks every source/destination pair. */
    for (k = 0; k < 8; ++k) for (i = 0; i < 8; ++i) for (j = 0; j < 8; ++j)
        if (reference[i][k] < INF_DISTANCE && reference[k][j] < INF_DISTANCE &&
            reference[i][k] + reference[k][j] < reference[i][j]) reference[i][j] = reference[i][k] + reference[k][j];
    for (i = 0; i < 8; ++i) for (j = 0; j < 8; ++j) {
        result = dijkstra_shortest_path(&graph, i, j);
        REQUIRE(result.distance == reference[i][j]);
        if (reference[i][j] < INF_DISTANCE) REQUIRE(valid_path(&graph, result, i, j));
        else REQUIRE(!result.found);
        result = astar_shortest_path(&graph, i, j);
        REQUIRE(result.distance == reference[i][j]);
        if (reference[i][j] < INF_DISTANCE) REQUIRE(valid_path(&graph, result, i, j));
        else REQUIRE(!result.found);
    }
    return 1;
}

static int load_coordinate_fixture(Graph *graph)
{
    char path[1024];
    if (!load_fixture(graph, "test_roads.csv")) return 0;
    if (snprintf(path, sizeof path, "%s/test_coordinates.csv", fixtures) >= (int)sizeof path) return 0;
    return load_coordinates(graph, path);
}

static int test_coordinates(void)
{
    Graph graph;
    REQUIRE(load_coordinate_fixture(&graph));
    REQUIRE(graph_has_complete_coordinates(&graph));
    REQUIRE(graph.locations[1].x == 30 && graph.locations[1].y == 40);
    return 1;
}

static int test_negative_coordinates(void)
{
    Graph graph;
    REQUIRE(load_coordinate_fixture(&graph));
    REQUIRE(graph.locations[4].x == -100 && graph.locations[4].y == -200);
    return 1;
}

static int test_unknown_coordinate(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("999,2,3\n10,4,5\n"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    REQUIRE(graph.count == 5 && graph.locations[0].x == 4);
    REQUIRE(!graph.locations[1].has_coordinates);
    return 1;
}

static int test_duplicate_coordinate(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("10,2,3\n10,9,9\n"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    REQUIRE(graph.locations[0].x == 2 && graph.locations[0].y == 3);
    return 1;
}

static int test_coordinate_fields(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("10,2\n20,2,3,4\n30,,4\n40,1,\n50,1,2\n"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    REQUIRE(!graph.locations[0].has_coordinates && !graph.locations[1].has_coordinates);
    REQUIRE(!graph.locations[2].has_coordinates && !graph.locations[3].has_coordinates);
    REQUIRE(graph.locations[4].has_coordinates);
    return 1;
}

static int test_coordinate_range(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("10,1.5,2\n20,1000001,0\n30,0,-1000001\n40,99999999999999999999999,0\n50,0,NaN\n"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    for (int i = 0; i < graph.count; ++i) REQUIRE(!graph.locations[i].has_coordinates);
    REQUIRE(write_fixture("10,-1000000,1000000\n"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    REQUIRE(graph.locations[0].x == -COORDINATE_LIMIT && graph.locations[0].y == COORDINATE_LIMIT);
    return 1;
}

static int test_coordinate_bom(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, NULL));
    REQUIRE(write_fixture("\xEF\xBB\xBF" " 10 , -2 , +3 \n\n20,0,0"));
    REQUIRE(load_coordinates(&graph, "route-test-generated.csv"));
    REQUIRE(graph.locations[0].x == -2 && graph.locations[0].y == 3 && graph.locations[1].has_coordinates);
    return 1;
}

static int test_missing_coordinates(void)
{
    Graph graph;
    REQUIRE(load_fixture(&graph, "test_roads.csv"));
    REQUIRE(!graph_has_complete_coordinates(&graph));
    REQUIRE(!load_coordinates(&graph, "route-test-missing-dir/coordinates.csv"));
    REQUIRE(!astar_shortest_path(&graph, 0, 1).found);
    REQUIRE(dijkstra_shortest_path(&graph, 0, 1).distance == 5);
    return 1;
}

static int test_euclidean(void)
{
    Location a = {0}, b = {0};
    b.x = 3; b.y = 4;
    REQUIRE(euclidean_distance(&a, &b) == 5.0);
    a.x = a.y = -COORDINATE_LIMIT;
    b.x = b.y = COORDINATE_LIMIT;
    REQUIRE(fabs(euclidean_distance(&a, &b) - hypot(2000000.0, 2000000.0)) < 1e-8);
    return 1;
}

static int test_heuristic_goal(void)
{
    Graph graph;
    double h[MAX_LOCATIONS];
    REQUIRE(load_coordinate_fixture(&graph));
    REQUIRE(build_goal_heuristic(&graph, 3, h) > 0.0);
    REQUIRE(h[3] == 0.0);
    for (int i = 0; i < graph.count; ++i) REQUIRE(h[i] >= 0.0);
    return 1;
}

static int test_scaled_edges(void)
{
    Graph graph;
    double alpha, h[MAX_LOCATIONS];
    REQUIRE(load_coordinate_fixture(&graph));
    alpha = graph_heuristic_scale(&graph);
    REQUIRE(fabs(alpha - 0.1) < 1e-12);
    for (int goal = 0; goal < graph.count; ++goal) {
        REQUIRE(build_goal_heuristic(&graph, goal, h) > 0.0);
        for (int i = 0; i < graph.count; ++i) for (int j = 0; j < graph.count; ++j) {
            int weight = graph.adjacency[i][j];
            if (weight < INF_DISTANCE) {
                REQUIRE(alpha * euclidean_distance(&graph.locations[i], &graph.locations[j]) <= weight + 1e-12);
                REQUIRE(h[i] <= weight + h[j]);
            }
        }
    }
    return 1;
}

static int test_zero_scale(void)
{
    Graph graph;
    PathResult a, d;
    REQUIRE(load_coordinate_fixture(&graph));
    for (int i = 0; i < graph.count; ++i) graph.locations[i].x = graph.locations[i].y = 0;
    a = astar_shortest_path(&graph, 0, 3); d = dijkstra_shortest_path(&graph, 0, 3);
    REQUIRE(a.heuristic_scale == 0.0 && path_results_agree(&graph, &a, &d, 0, 3));
    REQUIRE(a.expanded_nodes == d.expanded_nodes && a.relaxed_edges == d.relaxed_edges);
    REQUIRE(memcmp(a.path, d.path, sizeof a.path) == 0);
    return 1;
}

static int test_astar_direct(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_coordinate_fixture(&graph));
    result = astar_shortest_path(&graph, 0, 1);
    REQUIRE(result.distance == 5 && result.path_length == 2);
    REQUIRE(valid_path(&graph, result, 0, 1));
    return 1;
}

static int test_astar_multistep(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_coordinate_fixture(&graph));
    result = astar_shortest_path(&graph, 0, 3);
    REQUIRE(result.distance == 15 && result.path_length == 4);
    REQUIRE(valid_path(&graph, result, 0, 3));
    return 1;
}

static int test_astar_same(void)
{
    Graph graph;
    PathResult result;
    REQUIRE(load_coordinate_fixture(&graph));
    result = astar_shortest_path(&graph, 4, 4);
    REQUIRE(result.found && result.distance == 0 && result.path_length == 1 && result.path[0] == 4);
    REQUIRE(result.expanded_nodes == 1 && result.relaxed_edges == 0);
    return 1;
}

static int test_astar_unreachable(void)
{
    Graph graph;
    PathResult a, d;
    REQUIRE(load_coordinate_fixture(&graph));
    a = astar_shortest_path(&graph, 0, 4); d = dijkstra_shortest_path(&graph, 0, 4);
    REQUIRE(!a.found && a.distance == INF_DISTANCE && a.path_length == 0);
    REQUIRE(a.expanded_nodes == 4 && path_results_agree(&graph, &a, &d, 0, 4));
    REQUIRE(!astar_shortest_path(&graph, 4, 0).found);
    return 1;
}

static int test_astar_invalid(void)
{
    Graph graph;
    REQUIRE(load_coordinate_fixture(&graph));
    REQUIRE(!astar_shortest_path(&graph, -1, 0).found);
    REQUIRE(!astar_shortest_path(&graph, 0, graph.count).found);
    REQUIRE(!astar_shortest_path(NULL, 0, 0).found);
    REQUIRE(!dijkstra_shortest_path(NULL, 0, 0).found);
    graph_init(&graph);
    REQUIRE(!astar_shortest_path(&graph, 0, 0).found);
    return 1;
}

static int test_search_metrics(void)
{
    Graph graph;
    PathResult d, a;
    graph_init(&graph);
    for (int i = 1; i <= 3; ++i) REQUIRE(graph_add_location(&graph, i, "Node", "Type"));
    REQUIRE(graph_add_road(&graph, 1, 2, 2));
    REQUIRE(graph_add_road(&graph, 1, 3, 9));
    REQUIRE(graph_add_road(&graph, 2, 3, 3));
    for (int i = 0; i < 3; ++i) graph.locations[i].has_coordinates = 1;
    d = dijkstra_shortest_path(&graph, 0, 2); a = astar_shortest_path(&graph, 0, 2);
    REQUIRE(d.expanded_nodes == 3 && d.relaxed_edges == 3);
    REQUIRE(a.expanded_nodes == 3 && a.relaxed_edges == 3 && a.distance == 5);
    return 1;
}

static int test_decoy(void)
{
    Graph graph;
    char path[1024];
    PathResult d, a;
    graph_init(&graph);
    (void)snprintf(path, sizeof path, "%s/astar_decoy_locations.csv", fixtures);
    REQUIRE(load_locations(&graph, path));
    (void)snprintf(path, sizeof path, "%s/astar_decoy_roads.csv", fixtures);
    REQUIRE(load_roads(&graph, path));
    (void)snprintf(path, sizeof path, "%s/astar_decoy_coordinates.csv", fixtures);
    REQUIRE(load_coordinates(&graph, path));
    d = dijkstra_shortest_path(&graph, 0, 2); a = astar_shortest_path(&graph, 0, 2);
    REQUIRE(path_results_agree(&graph, &d, &a, 0, 2) && a.distance == 20);
    REQUIRE(valid_path(&graph, a, 0, 2) && valid_path(&graph, d, 0, 2));
    REQUIRE(a.expanded_nodes < d.expanded_nodes);
    return 1;
}

static int test_comparison_validation(void)
{
    Graph graph;
    PathResult a, d;
    REQUIRE(load_coordinate_fixture(&graph));
    a = astar_shortest_path(&graph, 0, 3); d = dijkstra_shortest_path(&graph, 0, 3);
    REQUIRE(path_results_agree(&graph, &a, &d, 0, 3));
    ++a.distance;
    REQUIRE(!path_results_agree(&graph, &a, &d, 0, 3));
    a = d; a.path[1] = 4;
    REQUIRE(!path_results_agree(&graph, &a, &d, 0, 3));
    a = d; a.found = 0;
    REQUIRE(!path_results_agree(&graph, &a, &d, 0, 3));
    return 1;
}

static int test_different_equal_paths(void)
{
    Graph graph;
    PathResult a, d;
    graph_init(&graph);
    for (int i = 0; i < 4; ++i) {
        REQUIRE(graph_add_location(&graph, i + 1, "Node", "Type"));
        graph.locations[i].has_coordinates = 1;
    }
    REQUIRE(graph_add_road(&graph, 1, 2, 1)); REQUIRE(graph_add_road(&graph, 2, 4, 3));
    REQUIRE(graph_add_road(&graph, 1, 3, 2)); REQUIRE(graph_add_road(&graph, 3, 4, 2));
    graph.locations[0].x = 0; graph.locations[1].x = -1;
    graph.locations[2].x = 2; graph.locations[3].x = 2;
    d = dijkstra_shortest_path(&graph, 0, 3); a = astar_shortest_path(&graph, 0, 3);
    REQUIRE(d.distance == 4 && a.distance == 4 && d.path[1] != a.path[1]);
    REQUIRE(path_results_agree(&graph, &a, &d, 0, 3));
    return 1;
}

static int test_many_graphs_reference(void)
{
    Graph graph;
    int reference[12][12], i, j, k, seed;
    for (seed = 1; seed <= 20; ++seed) {
        graph_init(&graph);
        for (i = 0; i < 12; ++i) {
            REQUIRE(graph_add_location(&graph, i + 1, "Node", "Type"));
            graph.locations[i].x = (i * 7919 + seed * 3571) % 2000001 - 1000000;
            graph.locations[i].y = (i * i * 101 + seed * 2131) % 2000001 - 1000000;
            graph.locations[i].has_coordinates = 1;
        }
        for (i = 0; i < 11; ++i) for (j = i + 1; j < 11; ++j)
            if ((i * 13 + j * 7 + seed) % 4 != 0)
                REQUIRE(graph_add_road(&graph, i + 1, j + 1, (i * 31 + j * 17 + seed * 23) % 100 + 1));
        for (i = 0; i < 12; ++i) for (j = 0; j < 12; ++j) reference[i][j] = graph.adjacency[i][j];
        for (k = 0; k < 12; ++k) for (i = 0; i < 12; ++i) for (j = 0; j < 12; ++j)
            if (reference[i][k] < INF_DISTANCE && reference[k][j] < INF_DISTANCE &&
                reference[i][k] + reference[k][j] < reference[i][j])
                reference[i][j] = reference[i][k] + reference[k][j];
        for (j = 0; j < 12; ++j) {
            double h[MAX_LOCATIONS];
            (void)build_goal_heuristic(&graph, j, h);
            for (i = 0; i < 12; ++i) {
                PathResult a = astar_shortest_path(&graph, i, j), d = dijkstra_shortest_path(&graph, i, j);
                REQUIRE(a.distance == reference[i][j] && d.distance == reference[i][j]);
                REQUIRE(path_results_agree(&graph, &a, &d, i, j));
                if (a.found) {
                    REQUIRE(valid_path(&graph, a, i, j) && valid_path(&graph, d, i, j));
                    REQUIRE(h[i] <= reference[i][j]);
                }
            }
        }
    }
    return 1;
}

static int test_astar_large_chain(void)
{
    Graph graph;
    PathResult a;
    graph_init(&graph);
    for (int i = 0; i < MAX_LOCATIONS; ++i) {
        REQUIRE(graph_add_location(&graph, i + 1, "Node", "Type"));
        graph.locations[i].has_coordinates = 1;
        graph.locations[i].x = i * 40000 - 1000000;
        if (i > 0) REQUIRE(graph_add_road(&graph, i, i + 1, MAX_ROAD_DISTANCE));
    }
    a = astar_shortest_path(&graph, 0, MAX_LOCATIONS - 1);
    REQUIRE(a.distance == (MAX_LOCATIONS - 1) * MAX_ROAD_DISTANCE && a.path_length == MAX_LOCATIONS);
    REQUIRE(valid_path(&graph, a, 0, MAX_LOCATIONS - 1));
    return 1;
}

static int test_repeated_queries(void)
{
    Graph graph, original;
    PathResult first, second;
    REQUIRE(load_coordinate_fixture(&graph));
    original = graph;
    first = astar_shortest_path(&graph, 0, 3);
    (void)astar_shortest_path(&graph, 3, 0);
    (void)dijkstra_shortest_path(&graph, 0, 4);
    second = astar_shortest_path(&graph, 0, 3);
    REQUIRE(first.distance == second.distance && first.expanded_nodes == second.expanded_nodes &&
            first.relaxed_edges == second.relaxed_edges && memcmp(first.path, second.path, sizeof first.path) == 0);
    REQUIRE(memcmp(&graph, &original, sizeof graph) == 0);
    return 1;
}

static int test_campus_reference(void)
{
    Graph graph;
    int reference[MAX_LOCATIONS][MAX_LOCATIONS], i, j, k;
    char path[1024];
    graph_init(&graph);
    (void)snprintf(path, sizeof path, "%s/../../data/locations.csv", fixtures);
    REQUIRE(load_locations(&graph, path));
    (void)snprintf(path, sizeof path, "%s/../../data/roads.csv", fixtures);
    REQUIRE(load_roads(&graph, path));
    (void)snprintf(path, sizeof path, "%s/../../data/coordinates.csv", fixtures);
    REQUIRE(load_coordinates(&graph, path));
    REQUIRE(graph.count == 12 && graph_has_complete_coordinates(&graph));
    for (i = 0; i < graph.count; ++i) for (j = 0; j < graph.count; ++j) reference[i][j] = graph.adjacency[i][j];
    for (k = 0; k < graph.count; ++k) for (i = 0; i < graph.count; ++i) for (j = 0; j < graph.count; ++j)
        if (reference[i][k] < INF_DISTANCE && reference[k][j] < INF_DISTANCE &&
            reference[i][k] + reference[k][j] < reference[i][j]) reference[i][j] = reference[i][k] + reference[k][j];
    for (i = 0; i < graph.count; ++i) for (j = 0; j < graph.count; ++j) {
        PathResult a = astar_shortest_path(&graph, i, j), d = dijkstra_shortest_path(&graph, i, j);
        REQUIRE(a.distance == reference[i][j] && d.distance == reference[i][j]);
        REQUIRE(valid_path(&graph, a, i, j) && valid_path(&graph, d, i, j));
    }
    return 1;
}

static int test_no_geometric_edges(void)
{
    Graph graph;
    double h[MAX_LOCATIONS];
    PathResult a;
    graph_init(&graph);
    REQUIRE(graph_add_location(&graph, 1, "Isolated", "Type"));
    graph.locations[0].has_coordinates = 1;
    REQUIRE(graph_heuristic_scale(&graph) == 0.0);
    REQUIRE(build_goal_heuristic(&graph, 0, h) == 0.0 && h[0] == 0.0);
    a = astar_shortest_path(&graph, 0, 0);
    REQUIRE(a.found && a.distance == 0 && a.expanded_nodes == 1);
    return 1;
}

static void run_test(const char *name, int (*test)(void))
{
    if (test()) { ++passed; printf("[PASS] %s\n", name); }
    else { ++failed; printf("[FAIL] %s\n", name); }
}

int main(int argc, char *argv[])
{
    if (argc == 2) fixtures = argv[1];
    else if (argc != 1) { fputs("Usage: route_tests [FIXTURE_DIRECTORY]\n", stderr); return 1; }
    run_test("graph initialization", test_initialization);
    run_test("load locations", test_locations);
    run_test("load roads", test_roads);
    run_test("ID to index mapping", test_id_mapping);
    run_test("direct shortest path", test_direct_route);
    run_test("multi-step shortest path and reconstruction", test_multi_step);
    run_test("same location", test_same_location);
    run_test("disconnected graph", test_disconnected);
    run_test("isolated location", test_isolated);
    run_test("invalid indices and empty graph", test_invalid_indices);
    run_test("case-insensitive multiple matches", test_search);
    run_test("empty keyword", test_empty_search);
    run_test("invalid road API", test_invalid_road_api);
    run_test("invalid road CSV", test_invalid_road_csv);
    run_test("invalid location CSV", test_invalid_location_csv);
    run_test("duplicate roads keep shortest", test_duplicate_roads);
    run_test("summary counts each road once", test_summary);
    run_test("empty summary", test_empty_summary);
    run_test("location capacity", test_location_capacity);
    run_test("CSV capacity", test_csv_capacity);
    run_test("maximum-distance 50-vertex chain", test_max_distance_chain);
    run_test("64-bit total road length", test_large_summary);
    run_test("stable equal-cost paths", test_equal_routes);
    run_test("integer validation", test_integer_validation);
    run_test("overlong line recovery", test_long_line_recovery);
    run_test("BOM and whitespace", test_bom_and_whitespace);
    run_test("invalid text fields", test_bad_labels);
    run_test("missing files", test_missing_files);
    run_test("all pairs versus independent reference", test_reference_all_pairs);
    run_test("load complete coordinates", test_coordinates);
    run_test("negative coordinates", test_negative_coordinates);
    run_test("unknown coordinate ID", test_unknown_coordinate);
    run_test("duplicate coordinate keeps first", test_duplicate_coordinate);
    run_test("coordinate field count", test_coordinate_fields);
    run_test("coordinate integer and range validation", test_coordinate_range);
    run_test("coordinate BOM and whitespace", test_coordinate_bom);
    run_test("missing coordinates rejected by A star only", test_missing_coordinates);
    run_test("Euclidean distance and large coordinates", test_euclidean);
    run_test("goal zero and nonnegative heuristics", test_heuristic_goal);
    run_test("scaled edge bounds and heuristic consistency", test_scaled_edges);
    run_test("zero scale degenerates to Dijkstra", test_zero_scale);
    run_test("A star direct path", test_astar_direct);
    run_test("A star multi-step path", test_astar_multistep);
    run_test("A star same endpoint and metrics", test_astar_same);
    run_test("A star unreachable", test_astar_unreachable);
    run_test("A star invalid indices null and empty graph", test_astar_invalid);
    run_test("successful relaxations and expansion metrics", test_search_metrics);
    run_test("decoy graph strictly fewer expansions", test_decoy);
    run_test("comparison rejects invalid results", test_comparison_validation);
    run_test("different equal-cost routes accepted", test_different_equal_paths);
    run_test("20 graphs all pairs and admissibility versus Floyd", test_many_graphs_reference);
    run_test("A star maximum-distance 50-vertex chain", test_astar_large_chain);
    run_test("queries independent and graph immutable", test_repeated_queries);
    run_test("campus all pairs versus Floyd", test_campus_reference);
    run_test("no geometric edges use zero heuristic", test_no_geometric_edges);
    (void)remove("route-test-generated.csv");
    printf("\n%d tests passed, %d tests failed.\n", passed, failed);
    return failed ? 1 : 0;
}
