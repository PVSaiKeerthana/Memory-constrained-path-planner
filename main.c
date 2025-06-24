#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define no_of_nodes 32
#define max_conn 4
#define NO_PARENT 255

uint8_t adj_list[no_of_nodes][max_conn];    // 128 bytes - adjacency list
uint8_t neighbor_count[no_of_nodes];        // 32 bytes - neighbor counts  
uint8_t visited_bits[4];                    // 4 bytes - bit-packed visited flags
uint8_t parent[no_of_nodes];                // 32 bytes - parent tracking
uint8_t shared_buffer[no_of_nodes];         // 32 bytes - shared queue/path buffer

uint8_t *queue_ptr = shared_buffer;         // Points to queue during BFS
uint8_t *path_ptr = shared_buffer;          // Points to path after BFS

const uint8_t edges[][2] = {
    {0,1},{0,6},{0,10},{1,2},{1,11},{2,3},{2,4},{2,5},{6,7},{6,8},{6,9},{10,11},
    {10,24},{10,26},{11,12},{11,19},{12,13},{12,14},{14,15},{14,16},
    {16,17},{16,18},{18,19},{18,21},{19,20},{21,22},{21,23},{23,24},
    {23,30},{24,25},{26,27},{26,28},{28,29},{28,30},{30,31}
};

// Bit manipulation functions using pointers
void set_visited(uint8_t node) {
    *(visited_bits + (node >> 3)) |= (1 << (node & 7));
}

uint8_t is_visited(uint8_t node) {
    return (*(visited_bits + (node >> 3)) >> (node & 7)) & 1;
}

void clear_visited() {
    *((uint32_t*)visited_bits) = 0;  // Fixed: Added missing * and cast
}

// Build graph from edge list
void build_graph() {
    memset(neighbor_count, 0, no_of_nodes);
    uint8_t num_edges = sizeof(edges) / sizeof(edges[0]);
    for (uint8_t i = 0; i < num_edges; i++) {
        uint8_t a = edges[i][0], b = edges[i][1];
        if (neighbor_count[a] < max_conn) {
            *(adj_list[a] + neighbor_count[a]++) = b;
        }
        if (neighbor_count[b] < max_conn) {
            *(adj_list[b] + neighbor_count[b]++) = a;
        }
    }
}

// BFS pathfinding algorithm using pointers
uint8_t find_path(uint8_t start, uint8_t goal) {
    clear_visited();
    memset(parent, NO_PARENT, no_of_nodes);
    uint8_t front = 0, rear = 0;
    *(queue_ptr + rear++) = start;
    set_visited(start);
    while (front < rear) {
        uint8_t curr = *(queue_ptr + front++);
        if (curr == goal) return 1;
        uint8_t *neighbors = adj_list[curr];
        for (uint8_t i = 0; i < neighbor_count[curr]; i++) {
            uint8_t neighbor = *(neighbors + i);
            if (!is_visited(neighbor)) {
                set_visited(neighbor);
                *(parent + neighbor) = curr;
                *(queue_ptr + rear++) = neighbor;
            }
        }
    }
    return 0;
}

// Path reconstruction using pointer manipulation
uint8_t reconstruct_path(uint8_t start, uint8_t goal) {
    uint8_t len = 0, curr = goal;
    uint8_t temp_path[no_of_nodes];
    while (curr != NO_PARENT && len < no_of_nodes) {
        *(temp_path + len++) = curr;
        curr = *(parent + curr);
    }
    for (uint8_t i = 0; i < len; i++) {
        *(path_ptr + i) = *(temp_path + (len - 1 - i));
    }
    return len;
}

// Display path results
void print_path(uint8_t path_length) {
    printf("Shortest path: ");
    for (uint8_t i = 0; i < path_length; i++) {
        printf("%d", *(path_ptr + i));
        if (i < path_length - 1) printf(" -> ");
    }
    printf("\nPath length: %d steps\n", path_length - 1);
}
int main() {
    printf("=== RISC-V Path Planning Demo ===\n");
    build_graph();
    uint8_t start = 4;   // Change this value (0-31)
    uint8_t goal = 25;   // Change this value (0-31)
    
    printf("Finding path from node %d to node %d...\n", start, goal);
    
    if (find_path(start, goal)) {
        uint8_t path_length = reconstruct_path(start, goal);
        print_path(path_length);
        printf("SUCCESS: Path found!\n");
    } else {
        printf("ERROR: No path exists between nodes %d and %d\n", start, goal);
    }
    return 0;
}
