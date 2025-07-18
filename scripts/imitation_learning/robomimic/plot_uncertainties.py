import matplotlib.pyplot as plt
import numpy as np




def load_data(file):
    rollouts = []
    rollout = []
    labels = []
    all_uncertainties = []
    all_timesteps = []

    with open(stack_cube_uncertainties_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip().split()
            timestep = int(line[0])
            uncertainties = [float(f) for f in line[1:8]]
            label = line[-1]

            if timestep == 0: 
                if rollout:
                    rollouts.append(rollout)
                    rollout = []
                labels.append(True if label == 'True' else False)

            if timestep % 10 == 0:
                rollout.append((timestep, uncertainties))
                all_timesteps.append(timestep)
                all_uncertainties.append(uncertainties)

    # Add final rollout
    if rollout:
        rollouts.append(rollout)

    return rollouts, labels, all_uncertainties, all_timesteps


def plot_rollouts(rollouts, results_path):
    num_joints = 7
    for i, rollout in enumerate(rollouts):
        rollout_timesteps = [t for t, _ in rollout]
        rollout_uncertainties = [unc for _, unc in rollout]

        # Transpose the list of uncertainties so each sublist is for one joint
        rollout_uncertainties_per_joint = list(zip(*rollout_uncertainties))

        plt.figure(figsize=(10, 6))
        for joint_num in range(num_joints):
            plt.plot(rollout_timesteps, rollout_uncertainties_per_joint[joint_num], label=f"Joint {joint_num}")

        plt.title(f"Uncertainty per Joint for Rollout {i}")
        plt.xlabel('Timestep')
        plt.ylabel('Uncertainty Value')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f"{results_path}uncertainty_rollout_{i}_plot_stack_cube.png")
        plt.close()



def plot_joint_uncertainty(all_timesteps, all_uncertainties, labels):
    num_joints = 7
    label_idx = 0
    success_uncertainties = [[] for _ in range(num_joints)]
    failure_uncertainties = [[] for _ in range(num_joints)]
    used_timesteps = []

    for i, (timestep, uncertainties) in enumerate(zip(all_timesteps, all_uncertainties)):
        # Use the current rollout label
        if i > 0 and all_timesteps[i] < all_timesteps[i - 1]:
            label_idx += 1  # New rollout started

        label = labels[label_idx]
        used_timesteps.append(timestep)

        for joint_num in range(num_joints):
            if label:
                success_uncertainties[joint_num].append(uncertainties[joint_num])
                failure_uncertainties[joint_num].append(np.nan)
            else:
                success_uncertainties[joint_num].append(np.nan)
                failure_uncertainties[joint_num].append(uncertainties[joint_num])

    # Plot
    for joint_num in range(num_joints):
        plt.figure(figsize=(10, 6))
        plt.plot(used_timesteps, success_uncertainties[joint_num], label="Task Success", color='green')
        plt.plot(used_timesteps, failure_uncertainties[joint_num], label="Task Failure", color='red')

        plt.title(f"Uncertainties for Joint {joint_num}")
        plt.xlabel('Timestep')
        plt.ylabel('Uncertainty Value')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()

        plt.savefig(f"./docs/training_data/uncertainty_joint_{joint_num}_plot_stack_cube.png")
        plt.close()


stack_cube_results_path = "./docs/training_data/stack_cube/uncertainty_rollout_stack_cube/model_70/" 
stack_cube_uncertainties_path = "./docs/training_data/stack_cube/uncertainty_rollout_stack_cube/model_70/uncertainties.txt"


rollouts, labels, all_uncertainties, all_timesteps = load_data(stack_cube_uncertainties_path)

plot_rollouts(rollouts, stack_cube_results_path)





