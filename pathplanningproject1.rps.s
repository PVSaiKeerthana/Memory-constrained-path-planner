#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define no_of_nodes 32
#define max_conn 4
#define NO_PARENT 63     
#define DIR_MASK 0x03    

enum { NORTH=0, EAST=1, SOUTH=2, WEST=3 };
// Graph edges
const uint8_t edges[][4] = {
    {0,1,EAST,WEST}, {0,6,SOUTH,NORTH}, {0,10,NORTH,SOUTH}, {1,2,SOUTH,NORTH}, {1,11,NORTH,SOUTH},{2,3,EAST,WEST},
    {2,4,EAST,WEST}, {2,5,WEST,EAST}, {6,7,EAST, WEST},{6,8,SOUTH,NORTH}, {6,9,WEST,EAST},
    {10,11,EAST,WEST},{10,24,NORTH,SOUTH}, {10,26,WEST,EAST}, {11,12,EAST,WEST}, {11,19,NORTH,SOUTH},
    {12,13,NORTH,SOUTH}, {12,14,NORTH,SOUTH},{14,15,WEST,EAST}, {14,16,NORTH,SOUTH},{16,17,SOUTH,NORTH}, {16,18,WEST,EAST},
    {18,19,SOUTH,NORTH}, {18,21,WEST,EAST},{19,20,WEST,EAST},{21,22,SOUTH,NORTH}, {21,23,WEST,EAST},
    {23,24,SOUTH,NORTH}, {23,30,WEST,EAST}, {24,25,EAST,WEST}, {26,27,NORTH,SOUTH}, {26,28,NORTH,SOUTH},
    {28,29,EAST,WEST}, {28,30,NORTH, SOUTH}, {30,31,SOUTH,NORTH}   
};

// Data memory arrays
uint8_t adj_list[no_of_nodes][max_conn];       // 128 bytes
uint8_t adj_dir[no_of_nodes];                  // 32 bytes 
uint8_t neighbor_count[no_of_nodes];           // 32 bytes
uint8_t visited_bits[4];                       // 4 bytes 
uint8_t parent_packed[24];                     // 24 bytes
uint8_t shared_buffer[no_of_nodes];            // 32 bytes

#define SWITCHES_START_ADDR  0xF0000000
#define SWITCHES_GOAL_ADDR   0xF0000004
#define UART_TX_ADDR         0x10000004
#define IR_SENSOR_ADDR       0x1000000C

void set_dir(uint8_t node, uint8_t idx, uint8_t dir) {
    uint8_t mask = ~(DIR_MASK << (2*idx));
    adj_dir[node] = (adj_dir[node] & mask) | ((dir & DIR_MASK) << (2*idx));
}
uint8_t get_dir(uint8_t node, uint8_t idx) {
    return (adj_dir[node] >> (2*idx)) & DIR_MASK;
}

void set_parent(uint8_t node, uint8_t par) {
    uint16_t bitpos = node * 6;
    uint16_t byteidx = bitpos / 8;
    uint8_t bitoff = bitpos % 8;

    uint32_t val = parent_packed[byteidx] | (parent_packed[byteidx+1] << 8) | (parent_packed[byteidx+2] << 16);
    val &= ~((uint32_t)0x3F << bitoff);
    val |= ((par & 0x3F) << bitoff);

    parent_packed[byteidx] = val & 0xFF;
    parent_packed[byteidx+1] = (val >> 8) & 0xFF;
    parent_packed[byteidx+2] = (val >> 16) & 0xFF;
}
uint8_t get_parent(uint8_t node) {
    uint16_t bitpos = node * 6;
    uint16_t byteidx = bitpos / 8;
    uint8_t bitoff = bitpos % 8;

    uint32_t val = parent_packed[byteidx] | (parent_packed[byteidx+1] << 8) | (parent_packed[byteidx+2] << 16);
    return (val >> bitoff) & 0x3F;
}

// Switch reading
uint8_t read_switches(const char* prompt, uintptr_t addr, uint8_t index) {
    printf("%s", prompt);
    fflush(stdout);
    static uint8_t prev_val[2] = {0xFF, 0xFF};
    uint8_t val;
    while (1) {
        val = *(volatile uint8_t*)addr;
        if (val != prev_val[index]) {
            prev_val[index] = val;
            if (val < no_of_nodes) {
                return val;
            } else {
                printf("Invalid node value %d (must be 0-%d). Please update correctly.\n", val, no_of_nodes-1);
            }
        }
    }
}

// UART output simulation
void uart_tx_byte(uint8_t v) {
    printf("[UART OUT]: 0x%02X\n", v);
}

// IR sensor input simulation
uint8_t sensor_read() {
    return *(volatile uint8_t*)IR_SENSOR_ADDR;
}

void set_visited(uint8_t node) {
    visited_bits[node >> 3] |= 1 << (node & 7);
}
uint8_t is_visited(uint8_t node) {
    return (visited_bits[node >> 3] >> (node & 7)) & 1;
}
void clear_visited() {
    *((uint32_t*)visited_bits) = 0;
}

// Build adjacency list
void build_graph() {
    memset(neighbor_count, 0, sizeof(neighbor_count));
    memset(adj_dir, 0, sizeof(adj_dir));
    for (uint8_t i=0; i<(sizeof(edges)/sizeof(edges[0])); i++) {
        uint8_t a = edges[i][0], b = edges[i][1];
        uint8_t da = edges[i][2], db = edges[i][3];
        if (neighbor_count[a] < max_conn) {
            uint8_t idx = neighbor_count[a];
            adj_list[a][idx] = b;
            set_dir(a, idx, da);
            neighbor_count[a]++;
        }
        if (neighbor_count[b] < max_conn) {
            uint8_t idx = neighbor_count[b];
            adj_list[b][idx] = a;
            set_dir(b, idx, db);
            neighbor_count[b]++;
        }
    }
}

// BFS pathfinding algorithm
uint8_t find_path(uint8_t start, uint8_t goal) {
    clear_visited();
    memset(parent_packed, 0xFF, sizeof(parent_packed)); // NO_PARENT = 63 (0x3F) - all bits set
    uint8_t front=0, rear=0;
    shared_buffer[rear++] = start;
    set_visited(start);
    set_parent(start, NO_PARENT);
    while (front < rear) {
        uint8_t curr = shared_buffer[front++];
        if (curr == goal) return 1;
        for (uint8_t i=0; i<neighbor_count[curr]; i++) {
            uint8_t nbr = adj_list[curr][i];
            if (!is_visited(nbr)) {
                set_visited(nbr);
                set_parent(nbr, curr);
                shared_buffer[rear++] = nbr;
            }
        }
    }
    return 0;
}

// Path reconstruction
uint8_t reconstruct_path(uint8_t start, uint8_t goal) {
    uint8_t len = 0;
    uint8_t curr = goal;
    uint8_t temp_path[no_of_nodes];
    while (curr != NO_PARENT && len < no_of_nodes) {
        temp_path[len++] = curr;
        curr = get_parent(curr);
    }
    for (uint8_t i=0; i<len; i++) {
        shared_buffer[i] = temp_path[len-1-i];
    }
    return len;
}

// Direction string for printing
const char* dir_name(uint8_t dir) {
    static const char* dirs[4] = {"NORTH", "EAST", "SOUTH", "WEST"};
    return dirs[dir];
}

void encode_and_send_directions(uint8_t path_length) {
    uint8_t orientation = 0;
    printf("Movement instructions:\n  Start at node %d, facing %s\n", shared_buffer[0], dir_name(orientation));
    for (uint8_t i = 1; i < path_length; i++) {
        uint8_t from = shared_buffer[i-1], to = shared_buffer[i];
        uint8_t move_dir = 0;
        uint8_t found = 0;
        for (uint8_t k = 0; k < neighbor_count[from]; k++) {
            if (adj_list[from][k] == to) {
                move_dir = get_dir(from, k);
                found = 1;
                break;
            }
        }
        if (!found) {
            printf("Unknown edge in path!\n");
            uart_tx_byte(0xFD);
            return;
        }
        uart_tx_byte(move_dir);
        int8_t turn = (int8_t)move_dir - (int8_t)orientation;
        if (turn < -2) turn += 4;
        if (turn > 2) turn -= 4;
        if (turn == 1 || turn == -3) printf("  Turn RIGHT\n");
        else if (turn == -1 || turn == 3) printf("  Turn LEFT\n");
        else if (turn == 2 || turn == -2) printf("  Turn AROUND\n");
        printf("  Move %s to node %d\n", dir_name(move_dir), to);
        printf("  (Checking IR sensor alignment...)\n");
        uint8_t sensor_val = sensor_read();
        printf("    %s\n", sensor_val == 1 ? "Aligned with path (sensor)" : "Not aligned (sensor)");
        orientation = move_dir;
    }
    printf("Arrived at goal node %d, facing %s\n", shared_buffer[path_length - 1], dir_name(orientation));
}

void simulate_movement(uint8_t path_length) {
    printf("Robot would now execute path step by step \n");
}
// FSM states
enum {
    STATE_INIT,
    STATE_WAIT_INPUT,
    STATE_PLAN,
    STATE_MOVE,
    STATE_DONE
};

int main() {
    uint8_t state = STATE_INIT;
    uint8_t start = 0, goal = 0, path_len = 0, found = 0;

    build_graph();

    while (1) {
        switch (state) {
        case STATE_INIT:
            printf(" Path Planner: Set switches (0xF0000000 and 0xF0000004) for start and goal nodes. \n");
            state = STATE_WAIT_INPUT;
            break;

        case STATE_WAIT_INPUT:
            start = read_switches("Set start node switches (0xF0000000) and update:\n", SWITCHES_START_ADDR, 0);
            printf("\nStart node received: %d\n", start);
            goal = read_switches("Set goal node switches (0xF0000004) and update:\n", SWITCHES_GOAL_ADDR, 1);
            printf("\nGoal node received: %d\n", goal);
            if (start >= no_of_nodes || goal >= no_of_nodes) {
                printf("Invalid nodes! Enter values between 0 and 31.\n");
                state = STATE_WAIT_INPUT;
            } else {
                state = STATE_PLAN;
            }
            break;

        case STATE_PLAN:
            found = find_path(start, goal);
            if (found) {
                path_len = reconstruct_path(start, goal);
                state = STATE_MOVE;
            } else {
                uart_tx_byte(0xFE);
                printf("No path found between %d and %d.\n", start, goal);
                state = STATE_DONE;
            }
            break;

        case STATE_MOVE:
            printf("\nShortest path: ");
            for (uint8_t i = 0; i < path_len; i++) {
                printf("%d", shared_buffer[i]);
                if (i < path_len - 1) printf(" -> ");
            }
            printf("\nPath length: %d steps\n", path_len - 1);
            encode_and_send_directions(path_len);
            simulate_movement(path_len);
            uart_tx_byte(0xFF);
            state = STATE_DONE;
            break;

        case STATE_DONE:
            printf("FSM in DONE state (halted).\n");
            while (1);
            break;
        }
    }
    return 0;
}
