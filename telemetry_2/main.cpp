extern "C"{
    #include "fake_receiver.h"
}
#include <string.h>
#include <iostream>
#include <string>
#include <chrono>
#include <unordered_map>

using namespace std;
using namespace chrono;


enum State {
    IDLE,
    RUN
};
struct Message {
    uint16_t id;
    uint8_t payload[8] = {0};
    uint8_t payload_size;   // Payload size in byte
};
struct Statistics {
    uint32_t tot_messages;
    uint64_t last_message_time;
    double mean_time;
};

const uint16_t START_STOP_ID = 0x0A0;
const uint16_t START_CODE1   = 0x6601;
const uint16_t START_CODE2   = 0xFF01;
const uint16_t STOP_CODE     = 0x66FF;

const string can_path    = "../candump.log";
const string stats_dir   = "../statistics";
const string logs_dir    = "../logs";

static State current_state;
static uint64_t start_time;
static uint64_t start_session_time;
static double clock_precision;     // Precision of the clock in seconds

static FILE *log_file, *stats_file;
static unordered_map<uint16_t, Statistics> stats;


void parse_message(const char message[MAX_CAN_MESSAGE_SIZE], Message & msg);
string format_time(uint64_t time);

void idle();
void run();

int create_log();
int save_stats();


void init() {
    current_state = IDLE;
    start_time = (uint64_t) high_resolution_clock::now().time_since_epoch().count();
    start_session_time = 0;
    clock_precision = (double) high_resolution_clock::period::num / high_resolution_clock::period::den;
}
int main(void){
    init();
    
    int is_open = open_can(can_path.c_str());
    if (is_open == 0) {
        while (true) {
            switch (current_state)
            {
                case IDLE:
                    idle();
                    break;
                case RUN:
                    run();
                    break;
            }
        }

        close_can();
    }
    else {
        cerr << "[Error]: Cannot open CAN!" << endl;
    }

    return 0;
}

void parse_message(const char message[MAX_CAN_MESSAGE_SIZE], Message & msg) {
    char *id_end;
    msg.id           = (uint16_t) strtoll(message, &id_end, 16);
    uint64_t payload = (uint64_t) strtoll(++id_end, NULL, 16);

    // Payload byte size
    msg.payload_size = (uint8_t) ((strlen(message) - 4) / 2);
    for (int i=0; i < msg.payload_size; i++)
    {
        // Get next byte
        msg.payload[msg.payload_size - i - 1] = (uint8_t) ((payload >> (i * 8)) & 0xFF);
    }
}
string format_time(uint64_t time) {
    const char time_format[] = "%Y-%m-%d_%H-%M-%S";
    char buffer[20];
    string out = "";

    // Get time in seconds
    time_t seconds = time * clock_precision;
    // Get time in milliseconds
    uint64_t ms    = (uint64_t)(time * (clock_precision * 1e3)) % 1000; 

    // Time to string formatting
    strftime(buffer, 20, time_format, localtime(&seconds));
    out += buffer;
    out += "_" + to_string(ms);

    return out;
}

void idle() {
    // Receive message
    char message[MAX_CAN_MESSAGE_SIZE];
    int read_bytes = can_receive(message);

    if (read_bytes == -1) {
        cerr << "[Error]: Error reading CAN!" << endl;
        return;
    }

    // Parse message
    Message msg;
    parse_message(message, msg);

    // Change state to run
    if (msg.id == START_STOP_ID && msg.payload_size == 2) {
        // Check if payload match with start code
        uint16_t code = (((uint16_t) msg.payload[0]) << 8) |
                        ((uint16_t) msg.payload[1]);
        if (code == START_CODE1 ||
            code == START_CODE2) {
            start_session_time = (uint64_t) high_resolution_clock::now().time_since_epoch().count();

            // Open log file
            int is_log_open = create_log();
            if (is_log_open == -1) {
                cerr << "[Error]: Error while creating log file!" << endl;
            }

            current_state = RUN;
        }
    }
}
void run() {
    // Receive message
    char message[MAX_CAN_MESSAGE_SIZE];
    int read_bytes = can_receive(message);
    
    // Get current time
    uint64_t now = (uint64_t) high_resolution_clock::now().time_since_epoch().count();

    if (read_bytes == -1) {
        cerr << "[Error]: Error reading CAN!" << endl;
        return;
    }

    // Parse message
    Message msg;
    parse_message(message, msg);
    
    if (msg.id == START_STOP_ID && msg.payload_size == 2) {
        // Check if payload match with start or stop code
        uint16_t code = (((uint16_t) msg.payload[0]) << 8) |
                        ((uint16_t) msg.payload[1]);
        // If start message is received do nothing
        if (code == START_CODE1 ||
            code == START_CODE2)
            return;
    
        // Change state to idle
        if (code == STOP_CODE) {
            // Close log file
            if (log_file != NULL) {
                if (fclose(log_file) == EOF)
                    cerr << "[Error]: Error while closing log file!" << endl;
            }

            // Save statistics to csv file
            if (save_stats() == -1) {
                cerr << "[Error]: Error while creating the csv file!" << endl;
            }
            
            current_state = IDLE;
            return;
        }
    }

    // Write to log
    if (log_file != NULL) {
        fprintf(log_file, "[%lu] %s\n", time(0), message);
        fflush(log_file);
    }

    // Update stats
    uint64_t time_ms = (uint64_t) (now * (clock_precision * 1e3));
    if (stats.find(msg.id) == stats.end()) {
        stats[msg.id] = {
            1,          // Number of messages
            time_ms,    // Time of the previous message
            0           // Average time between same messages
        };
    }
    else {
        uint64_t elapsed_time = time_ms - stats[msg.id].last_message_time;
        stats[msg.id].mean_time += (elapsed_time - stats[msg.id].mean_time) / (double) stats[msg.id].tot_messages;
        stats[msg.id].tot_messages++;
        stats[msg.id].last_message_time = time_ms;
    }
}

int create_log() {
    // Create log file with a human readable format
    string log_path = logs_dir + "/" + format_time(start_session_time) + ".log";

    log_file = fopen(log_path.c_str(), "w");
    if (log_file == NULL)
        return -1;

    return 0;
}
int save_stats() {
    // Create csv file with a human readable format 
    string stats_path = stats_dir + "/" + format_time(start_session_time) + ".csv";
    stats_file = fopen(stats_path.c_str(), "w");
    if (stats_file == NULL)
        return -1;

    // Write statistics to csv
    fprintf(stats_file, "ID;number_of_messages;mean_time\n");
    for (auto stat : stats) {
        uint16_t id = stat.first;
        Statistics & s = stat.second;
        uint32_t tot_messages = s.tot_messages;
        double mean_time = s.mean_time;

        fprintf(stats_file, "%03x;%lu;%.2f\n", id, tot_messages, mean_time);
    }
    fflush(stats_file);

    if (fclose(stats_file) == EOF)
        return -1;
    return 0;
}