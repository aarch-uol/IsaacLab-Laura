from isaaclab.app import AppLauncher
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import gym
import chills.tasks
print("[DEBUG] Registered:", list(gym.envs.registry.keys()))

# env = gym.make("FrankaIkLiftLabware-v0")
# env2 = gym.make("FrankaIkLiftLabware") 