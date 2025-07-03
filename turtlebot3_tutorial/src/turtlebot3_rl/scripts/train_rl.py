#!/usr/bin/env python3
"""
TurtleBot3 Reinforcement Learning Training Script
Uses Stable Baselines3 with PyTorch backend
"""

import os
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.monitor import Monitor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class TurtleBot3Env(gym.Env):
    """Custom TurtleBot3 Gym Environment"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS2 node
        rclpy.init()
        self.node = Node('turtlebot3_rl_env')
        
        # Action space: 0=forward, 1=turn_left, 2=turn_right, 3=stop
        self.action_space = gym.spaces.Discrete(4)
        
        # Observation space: [front_distance, left_distance, right_distance]
        self.observation_space = gym.spaces.Box(
            low=0.0, high=3.5, shape=(3,), dtype=np.float32
        )
        
        # ROS2 setup
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Reset service (for Gazebo)
        self.reset_client = self.node.create_client(Empty, '/reset_world')
        
        # Environment state
        self.laser_data = None
        self.position = None
        self.last_position = None
        self.episode_steps = 0
        self.max_episode_steps = 1000
        
        self.node.get_logger().info('TurtleBot3 RL Environment initialized!')
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_data = np.array(msg.ranges)
        # Replace inf values with max range
        self.laser_data = np.where(np.isinf(self.laser_data), 3.5, self.laser_data)
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.position = msg.pose.pose.position
    
    def get_observation(self):
        """Get current observation"""
        if self.laser_data is not None:
            # Get distances in front, left, and right
            front = np.mean(self.laser_data[350:360] + self.laser_data[0:10])
            left = np.mean(self.laser_data[80:100])
            right = np.mean(self.laser_data[260:280])
            
            # Normalize to [0, 1]
            obs = np.array([front, left, right], dtype=np.float32)
            obs = np.clip(obs, 0.0, 3.5) / 3.5
            
            return obs
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)
    
    def calculate_reward(self, action):
        """Calculate reward based on current state"""
        reward = 0.0
        done = False
        
        if self.laser_data is not None:
            # Get minimum distance to obstacle
            min_distance = np.min(self.laser_data)
            
            # Collision penalty
            if min_distance < 0.15:
                reward = -200.0
                done = True
                self.node.get_logger().info('Collision detected!')
            
            # Distance reward (encourage staying away from obstacles)
            elif min_distance < 0.5:
                reward = -50.0
            else:
                reward = 10.0
            
            # Action rewards
            if action == 0:  # Forward
                reward += 5.0
            elif action in [1, 2]:  # Turn
                reward += 1.0
            else:  # Stop
                reward -= 1.0
        
        # Episode length penalty
        if self.episode_steps >= self.max_episode_steps:
            done = True
            reward += 50.0  # Bonus for surviving
        
        return reward, done
    
    def step(self, action):
        """Execute one step in the environment"""
        # Execute action
        twist = Twist()
        
        if action == 0:  # Forward
            twist.linear.x = 0.2
        elif action == 1:  # Turn left
            twist.angular.z = 0.5
        elif action == 2:  # Turn right
            twist.angular.z = -0.5
        else:  # Stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)
        
        # Wait for sensor data
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Get observation and reward
        obs = self.get_observation()
        reward, done = self.calculate_reward(action)
        
        self.episode_steps += 1
        
        info = {
            'episode_steps': self.episode_steps,
            'min_distance': np.min(self.laser_data) if self.laser_data is not None else 0.0
        }
        
        return obs, reward, done, False, info
    
    def reset(self, seed=None, options=None):
        """Reset the environment"""
        super().reset(seed=seed)
        
        # Stop robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Reset Gazebo world
        if self.reset_client.wait_for_service(timeout_sec=5.0):
            request = Empty.Request()
            self.reset_client.call_async(request)
        
        # Reset episode variables
        self.episode_steps = 0
        self.last_position = None
        
        # Wait for sensor data
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        obs = self.get_observation()
        info = {'episode_steps': self.episode_steps}
        
        return obs, info
    
    def close(self):
        """Clean up resources"""
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    """Main training function"""
    print("Starting TurtleBot3 RL Training with PyTorch backend...")
    print(f"PyTorch version: {torch.__version__}")
    print(f"CUDA available: {torch.cuda.is_available()}")
    
    # Create environment
    env = TurtleBot3Env()
    env = Monitor(env)
    
    # Create model
    model = PPO(
        'MlpPolicy',
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        tensorboard_log="./tensorboard_logs/"
    )
    
    # Create callbacks
    eval_callback = EvalCallback(
        env,
        best_model_save_path='./logs/',
        log_path='./logs/',
        eval_freq=10000,
        deterministic=True,
        render=False
    )
    
    stop_callback = StopTrainingOnRewardThreshold(
        reward_threshold=200.0,
        verbose=1
    )
    
    # Train model
    try:
        model.learn(
            total_timesteps=100000,
            callback=[eval_callback, stop_callback],
            tb_log_name="turtlebot3_ppo"
        )
        
        # Save final model
        model.save("turtlebot3_ppo_final")
        print("Training completed!")
        
    except KeyboardInterrupt:
        print("Training interrupted by user")
    finally:
        env.close()


if __name__ == "__main__":
    main()