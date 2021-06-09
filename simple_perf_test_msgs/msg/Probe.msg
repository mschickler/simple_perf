# Counter that is incremented each time a probe is sent in order to detect missing messages
uint64 sequence

# The time, in nanoseconds, at which the message was constructed.
# This time is retrieved from the std::chrono::high_resolution_clock
uint64 timestamp

# A dynamically-sized array of data used to vary the size of the probe message
uint8[] payload
