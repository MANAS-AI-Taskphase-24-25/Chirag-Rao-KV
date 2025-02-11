# ROS 2 Simple publisher and subscriber project
## Task 5.0
Timothy is lost among a multitude of colors and needs to talk to Bob, create a simple Chat interface using minimal publisher and subscriber nodes in ROS2 to facilitate the same.

## Implementation

The task is to create a simple chat interface using ros2 publisher and subscriber nodes.
You need to use ROS2 publisher and subscriber to implement this functionality
Implement it using C++(preferably) or Python
You should be able to send multiple messages without waiting for a reply.

## Node : ros2 node list
These are the workig nodes :
```
/bob_node 

/rqt_gui_py_node_41453

/tim_node
```

## bob_node
This node is responsible to take input from the user and publish it to its subscriber.

## tim_node
This node is subscriber to bob_node, it recieves the message sent by the publisher node.

Both the nodes are connected by the TOPIC interact
## Topic : ros2 topic list
```
/interact

/parameter_events

/response

/rosout
```

The interact topic connects the two nodes.

## rqt_graph: rqt_graph
This can be visualized using the graph.

![rqt graph](rqt_graph.png)

# Execution
## bob_node: ros2 run my_package bob_node
```

Enter message to be sent to TIM (colour): red and blue

[INFO] [1739222379.733922446] [bob_node]: SENT TO TIM: 'red and blue'

Enter message to be sent to TIM (colour): green and yellow

[INFO] [1739222387.091877796] [bob_node]: SENT TO TIM: 'green and yellow'

Enter message to be sent to TIM (colour): orange

[INFO] [1739222401.952666619] [bob_node]: SENT TO TIM: 'orange'

Enter message to be sent to TIM (colour): 
```


## tim_node: ros2 run my_package tim_node
```
[INFO] [1739222379.734675606] [tim_node]: Getting colour: 'red and blue'

[INFO] [1739222379.734924360] [tim_node]: ->'Acknowledged! Which colour next? 


[INFO] [1739222387.092353227] [tim_node]: Getting colour: 'green and yellow'

[INFO] [1739222387.092449362] [tim_node]: ->'Acknowledged! Which colour next? 


[INFO] [1739222401.953264566] [tim_node]: Getting colour: 'orange'

[INFO] [1739222401.953390559] [tim_node]: ->'Acknowledged! Which colour next? 

```
