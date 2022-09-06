# Network QoS Measurement with
# - most recent partial latencies and total latency
# - mean/median/std/min/max of partial latencies and total latency over certain window

float32 total_latency
float32[] latencies

uint32 metrics_window

float32 total_latency_mean
float32[] latency_means

float32 total_latency_median
float32[] latency_medians

float32 total_latency_std
float32[] latency_stds

float32 total_latency_min
float32[] latency_mins

float32 total_latency_max
float32[] latency_maxs