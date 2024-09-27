#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "time_sync.h"

// Function to print current system time and return total time in microseconds
long print_current_time() {
    struct timeval current_time;

    // Get the current time
    gettimeofday(&current_time, NULL);

    // Print current time in seconds and microseconds
    printf("Current time: %ld seconds and %ld microseconds\n", 
           (long)current_time.tv_sec, 
           (long)current_time.tv_usec);
    
    // Calculate total time in microseconds since epoch
    long total_microseconds = (current_time.tv_sec * 1000000) + current_time.tv_usec;
    printf("Total time in microseconds since epoch: %ld µs\n", total_microseconds);

    return total_microseconds;
}

// Function to get the current system time in microseconds as a double
double get_system_time_us() {
    struct timeval current_time;

    // Get the current time
    gettimeofday(&current_time, NULL);

    // Convert time to microseconds and return as double
    double time_in_us = (double)(current_time.tv_sec) * 1e6 + (double)(current_time.tv_usec);
    return time_in_us;
}

// Function to calculate time difference in seconds
double time_diff_second(struct timeval start, struct timeval end) {
    return (double)(end.tv_sec - start.tv_sec) + (double)(end.tv_usec - start.tv_usec) / 1000000.0;
}

// Function to calculate frequency of updates
double calculate_frequency(struct timeval *last_update, int *update_count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current time

    // Calculate the time difference since the last update
    double elapsed_time = time_diff_second(*last_update, current_time);

    // Calculate frequency (updates per second)
    double frequency = 0.0;
    if (elapsed_time > 0) {
        frequency = (double)(*update_count) / elapsed_time; // Hz
    }

    // Update last_update time and reset update count
    *last_update = current_time;
    *update_count = 0; // Reset the count after calculating frequency

    return frequency;
}


// Function to calculate time difference in microseconds
double time_diff_microsecond(struct timeval start, struct timeval end) {
    return (double)(end.tv_sec - start.tv_sec) * 1000000.0 + (double)(end.tv_usec - start.tv_usec);
}

// Function to initialize the SyncSystem
void init_sync_system(SyncSystem *sys, double clock_hz) {
    sys->count = 0;
    sys->clock_hz = clock_hz;
    sys->sync_index = 0;
    sys->sample_count = 0;
    for (int i = 0; i < MAX_SAMPLES; i++) {
        sys->sync_times[i].tv_sec = 0;
        sys->sync_times[i].tv_usec = 0;
        sys->sync_counts[i] = 0;
    }
}

// Function to synchronize system time with the given count
void synchronize_time(SyncSystem *sys, double count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current system time

    // Store the system time and count in the circular buffer
    sys->sync_times[sys->sync_index] = current_time;
    sys->sync_counts[sys->sync_index] = count;
    
    sys->sync_index = (sys->sync_index + 1) % MAX_SAMPLES;
    if (sys->sample_count < MAX_SAMPLES) {
        sys->sample_count++;
    }
}

// Function to smooth the synchronization results
double smooth_time(SyncSystem *sys) {
    double sum_counts = 0;
    for (int i = 0; i < sys->sample_count; i++) {
        sum_counts += sys->sync_counts[i];
    }
    return sum_counts / sys->sample_count; // Return the average count
}

// Function to estimate system time based on a new count
double estimate_time(SyncSystem *sys, double count) {
    if (sys->sample_count == 0) {
        return -1; // No sync data available
    }

    // Use the most recent sync data for estimation
    int recent_index = (sys->sync_index - 1 + MAX_SAMPLES) % MAX_SAMPLES;
    struct timeval sync_time = sys->sync_times[recent_index];
    double sync_count = sys->sync_counts[recent_index];

    // Calculate time difference between the given count and the synced count
    double count_diff = count - sync_count;
    
    // Convert count difference to time difference (in microseconds)
    double estimated_time_diff = (count_diff / sys->clock_hz) * 1000000.0; // Convert to µs

    // Calculate the estimated system time
    struct timeval estimated_time = sync_time;
    estimated_time.tv_usec += estimated_time_diff;

    // Handle overflow in microseconds field
    while (estimated_time.tv_usec >= 1000000) {
        estimated_time.tv_sec += 1;
        estimated_time.tv_usec -= 1000000;
    }

    // Return the estimated system time in microseconds since epoch
    return estimated_time.tv_sec * 1000000.0 + estimated_time.tv_usec;
}


#if 0
// Function to calculate error between the estimated time and actual system time
double calculate_error(SyncSystem *sys, double count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current system time

    // Estimate the system time for the given count
    double estimated_time = estimate_time(sys, count);

    // If estimation failed
    if (estimated_time < 0) {
        return -1;
    }

    // Get the actual current time in microseconds since epoch
    double actual_time = current_time.tv_sec * 1000000.0 + current_time.tv_usec;

    // Calculate and return the error (difference between actual and estimated time)
    return actual_time - estimated_time;
}
#else
// Function to calculate error between the estimated time and actual system time
double calculate_error(SyncSystem *sys, double count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current system time

    // Estimate the system time for the given count
    double estimated_time = estimate_time(sys, count);

    // If estimation failed
    if (estimated_time < 0) {
        return -1;
    }

    // Get the actual current/last sync time in microseconds since epoch
    double actual_time = current_time.tv_sec * 1000000.0 + current_time.tv_usec;
    int recent_index = (sys->sync_index - 1 + MAX_SAMPLES) % MAX_SAMPLES;
    struct timeval sync_time = sys->sync_times[recent_index];
    double elapsed_time = actual_time - sync_time.tv_sec * 1000000.0 - sync_time.tv_usec;

    // Calculate the error (difference between actual and estimated time)
    double error = actual_time - estimated_time; 
    if (elapsed_time == error){ //just sync before error calculate_error
        error = 0;
    }
    //printf("actual %.2f estimate %.2f elapse %.2f err %.2f\n", actual_time, estimated_time, elapsed_time, error);
    
    // Calculate the change needed in parts per million (ppm) relative to clock frequency
    double percentage = error / elapsed_time * 100.0; // ppm relative to clock frequency

    // Print whether to increase or decrease the clock frequency
    if (error > 0) {
        printf("Decrease clock frequency by %.2f %% \n", percentage);
    } else if (error < 0) {
        printf("Increase clock frequency by %.2f %% \n", percentage);
    } else {
        printf("Clock frequency is accurate.\n");
    }

    return percentage;
}
#endif