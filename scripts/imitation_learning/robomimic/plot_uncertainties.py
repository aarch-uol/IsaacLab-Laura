import matplotlib.pyplot as plt


rollouts = []
rollout = []
labels = []
all_uncertainties = []
all_timesteps = []
with open("./docs/training_data/uncertainties.txt", 'r') as file:
    lines = file.readlines()
    for line in lines:
        line = line.strip().split()
        timestep = int(line[0])
        uncertainties = [float(f) for f in line[1:8]]
        label = line[-1]
        
        if timestep % 50 == 0:
            labels.append(True if label == 'True' else False)
            all_uncertainties.append(uncertainties)
            all_timesteps.append(timestep)

        if timestep == 0 and rollout:
            rollouts.append(rollout)
            rollout = []

        rollout.append((timestep, uncertainties))

if rollout:
    rollouts.append(rollout)

import numpy as np
num_joints = 7
for joint_num in range(num_joints):
    
    success_uncertainties = []
    failure_uncertainties = []

    for label, unc in zip(labels, all_uncertainties):
        if label:
            success_uncertainties.append(unc[joint_num])
            failure_uncertainties.append(np.nan)
        else:
            success_uncertainties.append(np.nan)
            failure_uncertainties.append(unc[joint_num])
    # success_uncertainties = [unc[joint_num] for label, unc in zip(labels, all_uncertainties) if label == True]
    # failure_uncertainties = [unc[joint_num] for label, unc in zip(labels, all_uncertainties) if label == False]

    # while len(success_uncertainties) != len(all_timesteps):
    #     success_uncertainties.append(np.nan)
    # while len(failure_uncertainties) != len(all_timesteps):
    #     failure_uncertainties.append(np.nan)

    plt.figure(figsize=(10, 6))

    plt.plot(all_timesteps, success_uncertainties, label=f"Task Success", color='green')
    plt.plot(all_timesteps, failure_uncertainties, label=f"Task Failure", color='red')
    
    plt.title(f"Uncertainties for Joint {joint_num}")
    plt.xlabel('Timestep')
    plt.ylabel('Uncertainty Value')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    plt.savefig(f"./docs/training_data/uncertainty_joint_{joint_num}_plot.png")
    plt.close()

# for rollout_num, rollout in enumerate(rollouts):
#     timesteps = [t for t, _ in rollout if t%10==0]
#     num_elements = 7

#     plt.figure(figsize=(10, 6))

#     for i in range(num_elements):
      
#         values = [unc[i] for t, unc in rollout if t%10==0]
#         plt.plot(timesteps, values, label=f'Uncertainty {i}')

#     plt.xlabel('Timestep')
#     plt.ylabel('Uncertainty value')
#     plt.title('Uncertainties per Timestep for Rollout 0')
#     plt.legend()
#     plt.grid(True)
#     plt.tight_layout()

#     plt.savefig(f"./docs/training_data/rollout_{rollout_num}_plot.png")



