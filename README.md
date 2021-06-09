This code implements the simplest of latency tests between a publisher and a subscriber.

Since the unique_ptr publish interface is used and the nodes are components
running in the same process, in theory the communications should occur with zero
copies.  However, the test result appears to indicate that a copy occurred
(i.e. the address of the underlying message received by the subscriber is changed).
