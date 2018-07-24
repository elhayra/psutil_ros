# number of bytes sent
uint64 bytes_sent

# number of bytes received
uint64 bytes_recv

# number of packets sent
uint64 packets_sent

# number of packets received
uint64 packets_recv

# total number of errors while receiving
uint64 errin

# total number of errors while sending
uint64 errout

# total number of incoming packets which were dropped
uint64 dropin

# total number of outgoing packets which were dropped (always 0 on macOS and BSD)
uint64 dropout
