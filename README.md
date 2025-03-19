# Perception Interfaces

This repository provides a set of ROS packages (ROS *and* ROS 2) with common messages and tools relating to the perception task in automated driving and C-ITS. The perception task here refers to both environment perception and self perception.

> [!IMPORTANT]  
> This repository is open-sourced and maintained by the [**Institute for Automotive Engineering (ika) at RWTH Aachen University**](https://www.ika.rwth-aachen.de/).  
> We cover a wide variety of research topics within our [*Vehicle Intelligence & Automated Driving*](https://www.ika.rwth-aachen.de/en/competences/fields-of-research/vehicle-intelligence-automated-driving.html) domain.  
> If you would like to learn more about how we can support your automated driving or robotics efforts, feel free to reach out to us!  
> :email: ***opensource@ika.rwth-aachen.de***

## Repository Structure

```
perception_interfaces
├── doc                             # Doxygen documentation  
├── perception_msgs                 # All perception related .msg files 
├── perception_msgs_rendering       # Rendering functions for RViz
├── perception_msgs_rviz_plugins    # RViz plugins (displays) that make use of the rendering functions  
├── perception_msgs_utils           # Utility (access) functions for the perception_msgs
└── tf2_perception_msgs             # Coordinate transformations for the perception_msgs
```

## Background

Perception is one central task in C-ITS. Entities such as vehicles and roadside units may be equipped with sensors with which they may perceive their environment and/or themselves. The computed perception data may then be forwarded to either other software modules within an entity or to other entities. To achieve this in ROS-based systems, the data is structured according to message files (*.msg). 

While ROS already provides [common_interfaces](https://github.com/ros2/common_interfaces) with various commonly used messages, these messages are insufficient to cover many types of data commonly exchanged in C-ITS, both within and across entities.

Especially for the communication between connected entities, [ETSI](https://en.wikipedia.org/wiki/ETSI) standardizes perception messages (and others) that are exchanged between the entities [here](https://forge.etsi.org/rep/ITS/asn1). These definitions present two major issues. First, they are based on a very generic description in [ASN.1](https://en.wikipedia.org/wiki/ASN.1) and therefore difficult to use. Second, they are overly complex for many use cases in research. 

The described circumstances motivate the creation of this repository, which contains a set of ROS packages with messages and tools that extend the insufficient [common_interfaces](https://github.com/ros2/common_interfaces) but remain much simpler and easier to handle than the perception messages defined by ETSI.

Anyone interested in using the more complex ETSI messages in ROS, despite the aforementioned difficulties, may be interested in [this repository](https://github.com/ika-rwth-aachen/etsi_its_messages). For communication *between* connected entities, this is the recommended solution for all messages that are already supported by the aforementioned repository.

## Overview of the perception_msgs

The perception_msgs contain simplified versions of two central messages defined by ETSI, the Cooperative Awareness Message (CAM) and the Collective Perception Message (CPM). The CAM and CPM correspond to the following messages defined in [perception_msgs](perception_msgs):

1. CAM → [EgoData.msg](perception_msgs/msg/EgoData.msg)
2. CPM → [ObjectList.msg](perception_msgs/msg/ObjectList.msg)

The [EgoData.msg](perception_msgs/msg/EgoData.msg) contains self perception data including:
- **Basic Vehicle Information**: Such as vehicle id, vehicle dimensions.
- **Dynamic State Estimate**: Such as pose, velocity, acceleration.
- **Self-reported Intent**: Planned route or trajectory.

The [ObjectList.msg](perception_msgs/msg/ObjectList.msg) contains environment perception data in the form of a list of [Objects](perception_msgs/msg/Object.msg) each of which is associated with:
- **Basic Object Information**: Such as object id, existence probability.
- **Dynamic State Estimate**: Such as pose, velocity, acceleration, object dimensions.
- **State Predictions**: Such as future pose, velocity, acceleration, object dimensions.

All other message files contained in [perception_msgs/msg](perception_msgs/msg/) are included by these two top-level message definitions.

## Dynamic States

Both the [EgoData.msg](perception_msgs/msg/EgoData.msg) and the [Objects.msg](perception_msgs/msg/Object.msg) contain a field of type [ObjectState.msg](perception_msgs/msg/ObjectState.msg). It contains information on an object that  
1. may change over time, or 
2. whose estimate may change over time. 

Of course, different information is available for perceived objects in the environment than for an entity that perceives its own state. That is why the [ObjectState.msg](perception_msgs/msg/ObjectState.msg) can contain different state models referenced by a [model_id](perception_msgs/msg/ObjectState.msg#L14). 

## Dynamic State Models

Anyone can define and add their own state model in a new message file under a new [model_id](perception_msgs/msg/ObjectState.msg#L14). The state model may contain all information relevant in a specific use case. At the moment, the following state models already exist:

1. [EGO.msg](perception_msgs/msg/EGO.msg) and [EGORWS.msg](perception_msgs/msg/EGORWS.msg) to be used in [EgoData.msg](perception_msgs/msg/EgoData.msg).
2. [ISCACTR.msg](perception_msgs/msg/ISCACTR.msg), [HEXAMOTION.msg](perception_msgs/msg/HEXAMOTION.msg) and [TRAFFICLIGHT.msg](perception_msgs/msg/TRAFFICLIGHT.msg) to be used for [Objects.msg](perception_msgs/msg/Object.msg) in [ObjectList.msg](perception_msgs/msg/ObjectList.msg).

A state model typically includes two types of state vectors: 

1. The [continuous_state](perception_msgs/msg/EGO.msg#L9) contains real-valued data (`float64`) such as pose, velocity, acceleration.
2. The [discrete_state](perception_msgs/msg/EGO.msg#L23) contains integer-valued data (`int64`) such as the current gear or the number of passengers.

In addition to the continuous and discrete state vectors, the state model should contain metadata:
1. [MODEL_ID](perception_msgs/msg/EGO.msg#L4): Can be freely chosen for new state models.
2. [CONTINUOUS_STATE_SIZE](perception_msgs/msg/EGO.msg#L5): Number of fields in the `continuous_state` vector.
3. [DISCRETE_STATE_SIZE](perception_msgs/msg/EGO.msg#L6): Number of fields in the `discrete_state` vector.

### Important: 

The `uint8` fields below the continuous and discrete state vectors of a state model define the index in the state vector under which a piece of information can be found. `uint8 X=0`, for example, indicates that the X position is at the 0th position in the vector, *not* that the X value is 0. The actual value in the vector is set in the code. Defining state vectors in this way allows us to do two things:

1. We do not need to know the location in the vector later when coding, because we can access elements of the vector like this:

    `double x_position = msg.continuous_state[EGO::X];`

2. We may change the location of the X position in the vector by changing the state model without the need to change any code, we may still access the X position of the EGO vehicle like this:

    `double x_position = msg.continuous_state[EGO::X];`

However, this convenience comes at a cost. We are able to receive data via the state vector that does not adhere to our definition and ROS would not be able to detect this out of the box, because the checksums of the message files may still be identical. Changes in the state model definitions, especially reordering, should therefore be handled with extreme care!

## Access functions

Working with hierarchical ROS messages, especially in C++, can be challenging sometimes due to the many nested subfields and certain requirements posed at how fields should be filled. For this reason, with [perception_msgs_utils](perception_msgs_utils), we provide a ROS package with access functions defined in C++ header files as well as Python equivalents. These enable developers to read and write fields of the perception_msgs more easily.

A list of all access functions can be found in [convenience_state_getters.h](perception_msgs_utils/include/perception_msgs_utils/impl/convenience_state_getters.h) and [convenience_state_setters.h](perception_msgs_utils/include/perception_msgs_utils/impl/convenience_state_setters.h).

In your C++ code, just include them like this

```c++
#include <perception_msgs_utils/object_access.hpp>
```

to use them. Example benefits include the ability to let the access functions automatically convert between quaternions and euler angles, or making sure that, e.g., yaw angles always lie between -PI and +PI.

The API documentation for the object access functions can be found [here](https://ika-rwth-aachen.github.io/perception_interfaces).

In your Python code, you can use the access functions like this:

```python
import perception_msgs_utils
```

## Coordinate Systems and Transformations

The coordinate system in which all subsequent fields of a message are given is defined in the top-level header, i.e. in [EgoData.msg](perception_msgs/msg/EgoData.msg#L11) and in [ObjectList.msg](perception_msgs/msg/ObjectList.msg#L16). Headers are a common ROS interface defined [here](https://github.com/ros2/common_interfaces/blob/rolling/std_msgs/msg/Header.msg). The definitions of the coordinate systems themselves may for example be determined by an [URDF](https://docs.ros.org/en/iron/Tutorials/Intermediate/URDF/URDF-Main.html) file.

Transformations between coordinate frames are often needed in robotics applications. There exists the [tf2](http://wiki.ros.org/tf2) ROS package ([GitHub](https://github.com/ros2/geometry2)) that provides a lot of functions and tools around coordinate frames and transformations. It is the de facto standard for related tasks in ROS. Since the package does not know how the messages files in this repository are structured, we need to define functions that allow us to integrate our messages into the existing framework provided by the tf2 ROS package. For this purpose, we need to especially write a specialized `doTransform()` function for our messages.

The implementation can be found in [tf2_perception_msgs](tf2_perception_msgs). Available implementations include the provided [EgoData.msg](perception_msgs/msg/EgoData.msg) with the [EGO.msg](perception_msgs/msg/EGO.msg) and [EGORWS.msg](perception_msgs/msg/EGORWS.msg) state models, and the [ObjectList.msg](perception_msgs/msg/ObjectList.msg) with the [ISCACTR.msg](perception_msgs/msg/ISCACTR.msg) and [HEXAMOTION.msg](perception_msgs/msg/HEXAMOTION.msg) state models.


## RViz Plugins

This repository provides ROS packages [perception_msgs_rendering](perception_msgs_rendering) and [perception_msgs_rviz_plugins](perception_msgs_rviz_plugins) that, together, allow to visualize [ObjectList.msg](perception_msgs/msg/ObjectList.msg) with the [ISCACTR.msg](perception_msgs/msg/ISCACTR.msg) and [HEXAMOTION.msg](perception_msgs/msg/HEXAMOTION.msg) state models and [EgoData.msg](perception_msgs/msg/EgoData.msg) with the [EGO.msg](perception_msgs/msg/EGO.msg) and [EGORWS.msg](perception_msgs/msg/EGORWS.msg) state models in RViz (only supported for ROS 2).

To further extend the object list, a workflow for visualizing new meshes has been established:

1. Import new meshes as .stl-files in Blender. (File → Import → Stl)
2. If not already correct, rotate Object so +x is the front and +z is the top of the object. (click on object → Object Properties → Transform → Rotation)
3. Set origin to center of surface. (right click on object → Set Origin → Origin to Center of Mass (Surface))
4. Export new file as .stl (File → Export → Stl) 

## Minimal implementation

Many of the fields in the messages are optional. If you use the messages in this repository, we recommend to provide at least

- the top-level header, and
- the object state(s)

where the object states should be initialized with [`initializeState()`](perception_msgs_utils/include/perception_msgs_utils/impl/init.h#L47) and then filled as much as possible.


## Acknowledgements

This research is accomplished within the research projects ”[autotech.agil](https://www.autotechagil.de/)” (FKZ 1IS22088A), ”[UNICAR*agil*](https://www.unicaragil.de/en/)” (FKZ 16EMO0284K), ”[6GEM](https://www.6gem.de/en/)” (FKZ 16KISK036K), and ”[AIthena](https://aithena.eu/)” (Grant Agreement No. 101076754). We acknowledge the financial support by the Federal Ministry of Education and Research of Germany (BMBF) and by the European Union under its Horizon Europe funding programme for research and innovation.