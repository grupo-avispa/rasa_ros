#!/usr/bin/env python3

# Copyright (c) 2025 Alberto J. Tudela Roldán
# Copyright (c) 2025 José Galeas Merchán
# Copyright (c) 2025 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rasa_msgs.action import Parse
from rasa_msgs.msg import Entity, Intent

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

import requests


class Rasa(Node):
    """Rasa node.

    This node is responsible for the interaction with the Rasa Open Source server.

    It includes the following features:
    - Action server to parse text using Rasa Open Source.

    Parameters
    ----------
    ip_address: str
        IP address of the Rasa Open Source server
    port: int
        Port of the Rasa Open Source server

    Actions
    ----------
    parse(rasa_msgs/Parse): Parse the text using Rasa Open Source

    Methods
    ----------
    __init__(self)
        Constructor
    get_params(self)
        Get parameters from the parameter server
    parse_callback(self, goal_handle)
        Callback for the parse action
    predict_intent(self, text, message_id)
        Get the intent from the Rasa Open Source server
    """

    def __init__(self):
        super().__init__('rasa_ros')

        # Get parameters
        self.get_params()

        # Create the action server
        self.action_server = ActionServer(self, Parse, 'parse', self.parse_callback)

    def get_params(self):
        """Get parameters from the parameter server."""
        # Declare and acquire parameters
        self.declare_parameter('ip_address', 'localhost')
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.get_logger().info(f'The parameter ip_address is set to: [{self.ip_address}]')

        self.declare_parameter('port', 5005)
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.get_logger().info(f'The parameter port is set to: [{self.port}]')

    def parse_callback(self, goal_handle) -> Parse.Result:
        """Parse callback.

        Parameters
        ----------
        goal_handle: rclpy.action.server.GoalHandle
            Goal handle of the parse action
        """
        goal = goal_handle.request
        self.get_logger().info(f'Executing goal: [{goal.text}]')
        [success, result] = self.predict_intent(goal.text, goal.message_id)

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    def predict_intent(self, text: str, message_id: str) -> tuple[bool, Parse.Result]:
        """Predicts the intent and entities of the message.

        Parameters
        ----------
        text: str
            Text to parse
        message_id: str
            Message ID

        Returns
        -------
        success: bool
            If the request was successful
        parse: rasa_msgs.msg.Parse.Result
            Parse result
        """
        # Initialize the success flag and parse
        success = False
        parse = Parse.Result()

        # Create the JSON payload
        payload = {'text': text, 'message_id': message_id}

        # Send a POST request to the Rasa Open Source server
        try:
            url = 'http://' + self.ip_address + ':' + str(self.port) + '/model/parse'
            response = requests.post(url, json=payload)

            # Check if the request was successful (or not)
            if response.status_code == 200:
                success = True
                # Extract the response data
                response_data = response.json()
                # Get the entities
                for entity_data in response_data['entities']:
                    entity = Entity()
                    entity.start = entity_data['start']
                    entity.end = entity_data['end']
                    entity.value = entity_data['value']
                    entity.entity = entity_data['entity']
                    entity.confidence = entity_data.get('confidence', 0.0)
                    parse.entities.append(entity)
                # Get the intent
                intent = Intent()
                intent.name = response_data['intent']['name']
                intent.confidence = response_data['intent']['confidence']
                parse.intent = intent
                # Get the ranking
                for ranking_data in response_data['intent_ranking']:
                    intent_ranking = Intent()
                    intent_ranking.name = ranking_data['name']
                    intent_ranking.confidence = ranking_data['confidence']
                    parse.intent_ranking.append(intent_ranking)
                # Get the text
                parse.text = response_data['text']

                self.get_logger().info(f'Intent predicted correctly: [{parse.text}]')
            elif response.status_code == 404:
                self.get_logger().error(f'Bad request: [{response.reason}]')
            elif response.status_code == 401:
                self.get_logger().error(f'User is not authenticated: [{response.reason}]')
            elif response.status_code == 403:
                self.get_logger().error(f'User is not authorized: [{response.reason}]')
            else:
                self.get_logger().error(f'Error requesting the intent: [{response.reason}]')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'Error requesting the intent: [{e}]')
        return success, parse


def main(args=None):
    rclpy.init(args=args)

    rasa = Rasa()

    rclpy.spin(rasa)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rasa.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
