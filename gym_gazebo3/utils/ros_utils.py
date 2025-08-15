import rclpy
from rclpy.node import Node
import time


class ROSUtils:
    """
    Utility functions for ROS2 operations
    """
    
    @staticmethod
    def wait_for_service(node, service_name, timeout=10.0):
        """Wait for a service to become available"""
        client = node.create_client(service_name)
        
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                return False
                
        return True
        
    @staticmethod
    def wait_for_topic(node, topic_name, topic_type, timeout=10.0):
        """Wait for a topic to start publishing"""
        received_msg = {'value': False}
        
        def callback(msg):
            received_msg['value'] = True
            
        subscription = node.create_subscription(
            topic_type, topic_name, callback, 10
        )
        
        start_time = time.time()
        while not received_msg['value'] and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        node.destroy_subscription(subscription)
        return received_msg['value']
        
    @staticmethod
    def spin_until_future_complete(node, future, timeout=10.0):
        """Spin node until future is complete or timeout"""
        start_time = time.time()
        
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            
        return future.done()