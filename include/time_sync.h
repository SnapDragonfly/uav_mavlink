#ifndef TIME_SYNC_FUNCTION_H
#define TIME_SYNC_FUNCTION_H

// Function to print current system time and return total time in microseconds
long print_current_time();

// Function to get the current system time in microseconds as a double
double get_system_time_us();

// Function to calculate time difference in seconds
double time_diff_second(struct timeval start, struct timeval end);

// Function to calculate time difference in microseconds
double time_diff_microsecond(struct timeval start, struct timeval end);

// Function to calculate frequency of updates
double calculate_frequency(struct timeval *last_update, int *update_count);


/*
 * Used for time estimate, according to
 * 1. counts
 * 2. clock
 * 3. systime
 */

#define MAX_SAMPLES 10

// Structure to hold synchronization data
typedef struct {
    double count;                  // Count value during synchronization
    double clock_hz;              // Clock frequency in Hz
    struct timeval sync_times[MAX_SAMPLES];  // System times for sync points
    double sync_counts[MAX_SAMPLES]; // Counts for sync points
    int sync_index;                // Index for the circular buffer
    int sample_count;              // Count of samples added
} SyncSystem;

// Function to initialize the SyncSystem
void init_sync_system(SyncSystem *sys, double clock_hz);

// Function to synchronize time with the given count
void synchronize_time(SyncSystem *sys, double count);

// Function to smooth the synchronization results
double smooth_time(SyncSystem *sys);

// Function to estimate system time based on count and clock frequency
double estimate_time(SyncSystem *sys, double count);

// Function to calculate the count based on the current system time
unsigned int calculate_timestamp(SyncSystem *sys);

// Function to calculate error statistics (mean error)
double calculate_error(SyncSystem *sys, double count);


#endif /* TIME_SYNC_FUNCTION_H */