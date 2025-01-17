# rasa_msgs

## Overview
This package contains messages and actions to interact with the [rasa_ros](../rasa_ros) package. It provides an interface for communication between ROS and Rasa Open Source, enabling the integration of natural language processing capabilities into robotic applications.

## Messages (.msg)
* [Entity](msg/Entity.msg): Defines a recognized entity in the text, including its name, value, and confidence in the original text.
* [Intent](msg/Intent.msg): Represents the detected intent in the text, including the intent name and prediction confidence.

## Actions (.action)
* [Parse](action/Parse.action): Action to parse a given text, returning the entities and intent detected by the Rasa model.