#!/usr/bin/env python3
"""
TurtleBot3 RL Model Evaluation Script
"""

import os
import numpy as np
import torch
from stable_baselines3 import PPO
from train_rl import TurtleBot3Env
import time


def evaluate_model(model_path, num_episodes=10):
    """Evaluate trained model"""
    print(f"Loading model from: {model_path}")
    
    # Load model
    model = PPO.load(model_path)
    
    # Create environment
    env = TurtleBot3Env()
    
    total_rewards = []
    total_steps = []
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0
        episode_steps = 0
        done = False
        
        print(f"\nEpisode {episode + 1}/{num_episodes}")
        
        while not done:
            # Get action from model
            action, _ = model.predict(obs, deterministic=True)
            
            # Execute action
            obs, reward, done, truncated, info = env.step(action)
            
            episode_reward += reward
            episode_steps += 1
            
            # Print progress
            min_distance = info.get('min_distance', 0.0)
            print(f"Step {episode_steps}: Action={action}, Reward={reward:.2f}, Min Distance={min_distance:.2f}")
            
            if done or truncated:
                break
            
            time.sleep(0.1)  # Small delay for visualization
        
        total_rewards.append(episode_reward)
        total_steps.append(episode_steps)
        
        print(f"Episode {episode + 1} completed: Reward={episode_reward:.2f}, Steps={episode_steps}")
    
    # Calculate statistics
    avg_reward = np.mean(total_rewards)
    std_reward = np.std(total_rewards)
    avg_steps = np.mean(total_steps)
    
    print(f"\nEvaluation Results:")
    print(f"Average Reward: {avg_reward:.2f} ± {std_reward:.2f}")
    print(f"Average Steps: {avg_steps:.1f}")
    print(f"Max Reward: {max(total_rewards):.2f}")
    print(f"Min Reward: {min(total_rewards):.2f}")
    
    env.close()
    return avg_reward, std_reward


def main():
    """Main evaluation function"""
    print("TurtleBot3 RL Model Evaluation")
    print("=" * 40)
    
    # Check for trained models
    model_paths = [
        "turtlebot3_ppo_final.zip",
        "logs/best_model.zip",
        "turtlebot3_ppo.zip"
    ]
    
    model_path = None
    for path in model_paths:
        if os.path.exists(path):
            model_path = path
            break
    
    if model_path is None:
        print("No trained model found. Please train a model first using train_rl.py")
        return
    
    try:
        avg_reward, std_reward = evaluate_model(model_path)
        print(f"\nFinal Score: {avg_reward:.2f} ± {std_reward:.2f}")
        
    except KeyboardInterrupt:
        print("\nEvaluation interrupted by user")
    except Exception as e:
        print(f"Error during evaluation: {e}")


if __name__ == "__main__":
    main()