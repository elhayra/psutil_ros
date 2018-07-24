# total physical memory.
uint64 total

# the memory that can be given instantly to processes 
# without the system going into swap. This is calculated
# by summing different memory values depending on the
# platform and it is supposed to be used to monitor actual
# memory usage in a cross platform fashion.
uint64 available

# memory used, calculated differently depending on the
# platform and designed for informational purposes only. 
# total - free does not necessarily match used.
uint64 used

# memory not being used at all (zeroed) that is readily 
# available; note that this doesn’t reflect the actual
# memory available (use available instead). total - used
# does not necessarily match free.
uint64 free

float32 percent
