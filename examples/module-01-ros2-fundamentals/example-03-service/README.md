# Example 03: Services (Request-Response Pattern)

## Learning Objectives

By completing this example, you will:
- Understand the service communication pattern in ROS 2
- Create a service server that processes requests
- Create a service client that sends requests and waits for responses
- Learn about synchronous vs. asynchronous communication
- Understand service definitions and request/response types

## Overview

Services provide synchronous, one-to-one communication in ROS 2. Unlike publish-subscribe (which is asynchronous and many-to-many), services follow a request-response pattern where a client sends a request and blocks until receiving a response from the server.

**Key Concepts:**
- **Service Server**: A node that provides a service and processes requests
- **Service Client**: A node that calls a service and waits for a response
- **Service Type**: Defines the request and response message structures
- **Synchronous Communication**: Client blocks until response is received

## When to Use Services

**Use Services For:**
- Requesting specific computations or actions
- Querying current state or configuration
- Triggering one-time operations
- Request-reply interactions where you need confirmation

**Use Topics For:**
- Continuous data streams (sensor data, state updates)
- Broadcasting information to multiple subscribers
- Fire-and-forget messaging

## Prerequisites

- ROS 2 Humble installed
- Completed Example 01 (Publisher-Subscriber)
- Completed Module 01 Chapter 04

## Directory Structure

```
example-03-service/
├── README.md           # This file
├── server.py           # Service server implementation
├── client.py           # Service client implementation
└── package.xml         # Package dependencies (optional)
```

## Code Walkthrough

### Service Type: `example_interfaces/AddTwoInts`

This example uses the built-in `AddTwoInts` service type:

**Request:**
```
int64 a
int64 b
```

**Response:**
```
int64 sum
```

### Service Server (`server.py`)

The server listens for addition requests and returns the sum:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response
```

**Key Components:**
1. **Service Creation**: `create_service(srv_type, service_name, callback)`
2. **Callback Function**: Receives request, populates response, returns response
3. **Request/Response Objects**: Strongly typed data structures

### Service Client (`client.py`)

The client sends addition requests to the server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future
```

**Key Components:**
1. **Client Creation**: `create_client(srv_type, service_name)`
2. **Service Availability Check**: `wait_for_service(timeout_sec)`
3. **Asynchronous Call**: `call_async(request)` returns a future
4. **Future Handling**: Wait for future to complete and retrieve response

## Running the Example

### Terminal 1: Start the Service Server

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run the server
python3 server.py
```

Expected output:
```
[INFO] [minimal_service]: Service ready: add_two_ints
```

### Terminal 2: Run the Service Client

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run the client with arguments
python3 client.py 5 7
```

Expected output (client):
```
[INFO] [minimal_client_async]: Sending request: 5 + 7
[INFO] [minimal_client_async]: Result: 5 + 7 = 12
```

Expected output (server):
```
[INFO] [minimal_service]: Incoming request: 5 + 7
[INFO] [minimal_service]: 5 + 7 = 12
```

## Command Line Service Calls

You can also call services from the command line:

```bash
# List all available services
ros2 service list

# Get information about a service
ros2 service type /add_two_ints

# Call the service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 15}"
```

## Introspection Tools

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /add_two_ints

# Find services of a specific type
ros2 service find example_interfaces/srv/AddTwoInts

# Show service definition
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Exercises

### Exercise 1: Multiply Two Numbers
Create a new service for multiplying two numbers instead of adding them.

### Exercise 2: Input Validation
Modify the server to reject requests where `a` or `b` is negative.

### Exercise 3: Multiple Operations
Create a service that accepts an operation type (add, subtract, multiply, divide) and performs the requested operation.

### Exercise 4: Timeout Handling
Modify the client to handle cases where the service takes too long to respond.

### Exercise 5: Custom Service Type
Create a custom service definition for a calculator with request fields: `a`, `b`, `operation`.

## Common Issues

### Issue 1: "Service not available"
**Solution**: Ensure the server is running before starting the client.

### Issue 2: "Timeout waiting for service"
**Solution**: Verify the service name matches exactly (case-sensitive) between client and server.

### Issue 3: Client hangs indefinitely
**Solution**: Always implement timeout logic with `wait_for_service(timeout_sec)`.

### Issue 4: "Module 'example_interfaces' not found"
**Solution**: Install the package: `sudo apt install ros-humble-example-interfaces`

## Synchronous vs. Asynchronous Calls

### Synchronous (Blocking)

```python
# Blocks until response is received
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
response = future.result()
```

### Asynchronous (Non-Blocking)

```python
# Returns immediately with a future
future = client.call_async(request)
# Continue doing other work...
# Check later if future is complete
if future.done():
    response = future.result()
```

## Key Takeaways

1. **Synchronous Communication**: Client waits for server response
2. **One-to-One**: Each request is handled by exactly one server
3. **Typed Interfaces**: Request and response types are strictly defined
4. **Service Discovery**: Clients must wait for service availability
5. **Error Handling**: Always implement timeouts and failure handling

## Next Steps

- **Example 04**: Actions for long-running operations with feedback
- **Module 01 Chapter 05**: Deep dive into Actions
- **Custom Service Definitions**: Create your own .srv files

## References

- [ROS 2 Service Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Service Interfaces Documentation](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [example_interfaces Package](https://github.com/ros2/example_interfaces)
