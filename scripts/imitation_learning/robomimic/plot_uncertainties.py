import matplotlib.pyplot as plt
import numpy as np




def load_data(file):
    rollouts = []
    rollout = []
    labels = []
    all_uncertainties = []
    all_timesteps = []

    with open(file, 'r') as file:
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

            if timestep % 1 == 0:
                rollout.append((timestep, uncertainties))
                all_timesteps.append(timestep)
                all_uncertainties.append(uncertainties)

    # Add final rollout
    if rollout:
        rollouts.append(rollout)

    return rollouts, labels, all_uncertainties, all_timesteps


def plot_rollouts(rollouts, labels, results_path, duration=400, joints_to_plot=[_ for _ in range(6)]):
    num_joints = 7
    for i, rollout in enumerate(rollouts):
        rollout_timesteps = [t for t, _ in rollout]
        rollout_uncertainties = [unc for _, unc in rollout]

        # Transpose the list of uncertainties so each sublist is for one joint
        rollout_uncertainties_per_joint = list(zip(*rollout_uncertainties))

        plt.figure(figsize=(10, 6))
        plt.ylim(bottom=0)
        plt.ylim(top=1.0)
        for joint_num in range(num_joints):
            if joint_num in joints_to_plot:
                rollout_timesteps_to_plot = list(rollout_timesteps[:duration])
                rollout_uncertainties_per_joint_to_plot = list(rollout_uncertainties_per_joint[joint_num][:duration])
                
                # pad timesteps
                j = len(rollout_timesteps_to_plot)
                while len(rollout_timesteps_to_plot) < duration:
                    rollout_timesteps_to_plot.append(j + 1)
                    j += 1
                # pad uncertainties
                while len(rollout_uncertainties_per_joint_to_plot) < duration:
                    rollout_uncertainties_per_joint_to_plot.append(0)

                plotted_line = plt.plot(rollout_timesteps_to_plot, rollout_uncertainties_per_joint_to_plot, label=f"Joint {joint_num}")
                mean = np.mean(rollout_uncertainties_per_joint_to_plot)
                mean = [mean for _ in range(len(rollout_timesteps_to_plot))]
                plt.plot(rollout_timesteps_to_plot, mean, label=f"Joint {joint_num} mean uncertainty", linestyle='--', color=plotted_line[0].get_color()) 

        title = f"Uncertainty for joint 6 for: Rollout {i}, Model: {model_arch}, Task: {task}, {'Success' if labels[i] == True else 'Failure'}"
        plt.title(title)
        plt.xlabel('Timestep')
        plt.ylabel('Uncertainty Value')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f"{results_path}{title}.png")
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


model_arch = "BC_RNN_GMM"
task = "stack_cube" # stack_cube or pick_place
model_name = f"model2"

results_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/" 
uncertainties_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/uncertainties.txt"


rollouts, labels, all_uncertainties, all_timesteps = load_data(uncertainties_path)

plot_rollouts(rollouts, labels, results_path, duration=400, joints_to_plot=[6])


# model 2
# Successful trials: 1, out of 10 trials
# Success rate: 0.1
# Trial Results: [False, False, False, False, True, False, False, False, False, False]


# best 20% model: model_epoch_2290_best_validation_1421192.9875.pth
# Successful trials: 2, out of 10 trials
# Success rate: 0.2
# Trial Results: [False, False, False, False, False, False, True, False, False, True]

# best 50% model: model_epoch_898_best_validation_1302229.0125.pth
# Successful trials: 0, out of 10 trials
# Success rate: 0.0
# Trial Results: [False, False, False, False, False, False, False, False, False, False]


# best 100% model: model_epoch_1012_best_validation_1162128.08125.pth
# Successful trials: 4, out of 10 trials
# Success rate: 0.4
# Trial Results: [False, True, True, False, False, False, True, False, True, False]




