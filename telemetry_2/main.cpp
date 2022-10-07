extern "C"{
    #include "fake_receiver.h"
}
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <unordered_map>
#include <chrono>

using namespace std;
using namespace chrono;


const uint16_t START_STOP_ID = 0x0A0;     // Start and stop message ID
const uint64_t START_CODE1   = 0x6601;
const uint64_t START_CODE2   = 0xFF01;
const uint64_t STOP_CODE     = 0x66FF;

const char can_path[]    = "../candump.log";
const char csv_path[]    = "../statistics/stats.csv";
const char log_dir[]     = "../logs";
const char time_format[] = "%Y-%m-%d_%H-%M-%S";


enum State {
    IDLE,
    RUN
};

struct Message {
    uint16_t id;
    uint64_t payload;
};
struct Statistics {
    uint32_t tot_messages;
    uint64_t last_message_time;
    uint64_t total_time;
};

State current_state = IDLE;
unordered_map<uint16_t, Statistics> stats;

FILE *log_file;
uint64_t start_time;

void parse_message(const char message[MAX_CAN_MESSAGE_SIZE], Message & msg);
void idle();
void run();

int create_log();
int save_stats();


int main(void){
    int is_open = open_can(can_path);
    start_time = (uint64_t) high_resolution_clock::now().time_since_epoch().count();

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
    msg.id      = (uint16_t) strtol(message, &id_end, 16);
    msg.payload = (uint64_t) strtol(++id_end, NULL, 16);
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
    if (msg.id == START_STOP_ID &&
           (msg.payload == START_CODE1 ||
            msg.payload == START_CODE2)) {
        int is_log_open = create_log();
        if (is_log_open == -1) {
            cerr << "[Error]: Error while creating log file!" << endl;
        }

        current_state = RUN;
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
    
    // Write to log
    if (log_file != NULL) {
        fprintf(log_file, "[%lu] %s\n", time(0), message);
        fflush(log_file);
    }

    // Update stats
    uint64_t time_ms = now / 1e6;
    if (stats.find(msg.id) == stats.end()) {
        stats[msg.id] = {
            1,          // Number of messages
            time_ms,    // Time of the previous message
            0           // Total time elapsed
        };
    }
    else {
        stats[msg.id].tot_messages++;
        stats[msg.id].total_time += time_ms - stats[msg.id].last_message_time;
        stats[msg.id].last_message_time = time_ms;
    }

    // Change state to idle
    if (msg.id == START_STOP_ID &&
            msg.payload == STOP_CODE) {
        if (log_file != NULL) {
            if (fclose(log_file) == EOF)
                cerr << "[Error]: Error while closing log file!" << endl;
        }

        // Save statistics to csv file
        if (save_stats() == -1) {
            cerr << "[Error]: Error while creating the csv file!" << endl;
        }
        
        current_state = IDLE;
    }
}

int create_log() {
    // Create log file with a human readable format
    char log_path[28];
    char time_string[20];

    // Get current time
    uint64_t now = (uint64_t) high_resolution_clock::now().time_since_epoch().count();
    now /= (uint64_t) 1e6;
    now %= (uint64_t) 1000;
    time_t t = time(0);

    memset(log_path, 0, 28);

    // Convert time from unix timestamp to date-time format
    strftime(time_string, 20, time_format, localtime(&t));
    sprintf(log_path, "%s/%s_%lu.log", log_dir, time_string, now);
    log_file = fopen(log_path, "w");

    if (log_file == NULL)
        return -1;

    return 0;
}
int save_stats() {
    FILE *csv_stats = fopen(csv_path, "w");
    if (csv_stats == NULL)
        return -1;

    // Write statistics to csv
    fprintf(csv_stats, "ID;number_of_messages;mean_time\n");
    for (auto stat : stats) {
        uint16_t id = stat.first;
        Statistics & s = stat.second;
        uint32_t tot_messages = s.tot_messages;
        float mean_time = ((float) s.total_time) / ((float) tot_messages);

        fprintf(csv_stats, "%03x;%d;%.2f\n", id, tot_messages, mean_time);
    }
    fflush(csv_stats);

    if (fclose(csv_stats) == EOF)
        return -1;
    return 0;
}