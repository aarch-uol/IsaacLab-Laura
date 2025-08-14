import os
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import argparse
from scipy.stats import gaussian_kde
from scipy.interpolate import CubicSpline
from sklearn.metrics import roc_curve, auc

from isaaclab.utils.logging_helper import LoggingHelper


def process_rollout_logger(rollout_log_path):
    """
    for each rollout, return the timesteps that each subtask was completed
    """
    subtask_codes = {
        'APPR': '0',
        'GRASP': '1',
        'LIFT': '2',
        'FINISH': '4'
    }
    
    # Check if rollout log data for this rollout already exists, if so use it. If not, it means we are 
    # running a new rollout so get the log data from the original path and then copy it over to save it.
    try:
        with open(rollout_log_path, 'r') as file:
            print(f"Reading rollout log data from {rollout_log_path}")
            lines = file.readlines()
    except FileNotFoundError as filenotfounderror:
        with open(LoggingHelper().namefile, 'r') as file:
            print(f"Using original rollout logging path at {LoggingHelper().namefile}")
            raw_content = file.read()
            file.seek(0)
            lines = file.readlines()
            with open(rollout_log_path, 'w') as file:
                file.write(raw_content)

    

    object_goal_distances = []
    end_effector_distances_to_object = []
    subtask_rollouts = [] 

    rollout_number = -1
    current_timestep = -1
    for line in lines:
        if rollout_number > args.num_rollouts:
            break
        line = line.strip().split(':')
        
        if line[0] == 'X':
            rollout_number += 1
            current_timestep = -1

        if line[0] == 'S':
            current_timestep += 1
    
        # APPR subtask was completed successfully
        if line[0] == subtask_codes['APPR'] and line[2] == 'TRUE' and 'APPR' not in [subtask_name for rol_num, subtask_name, _ in subtask_rollouts if rol_num == rollout_number]:
            subtask_rollouts.append([rollout_number, 'APPR', current_timestep])

        if line[0] == subtask_codes['GRASP'] and line[2] == 'TRUE' and 'GRASP' not in [subtask_name for rol_num, subtask_name, _ in subtask_rollouts if rol_num == rollout_number]:
            subtask_rollouts.append([rollout_number, 'GRASP', current_timestep])
        
        if line[0] == subtask_codes['LIFT'] and line[2] == 'TRUE' and 'LIFT' not in [subtask_name for rol_num, subtask_name, _ in subtask_rollouts if rol_num == rollout_number]:
            subtask_rollouts.append([rollout_number, 'LIFT', current_timestep])

        if line[0] == subtask_codes['FINISH'] and line[2] == 'TRUE' and 'FINISH' not in [subtask_name for rol_num, subtask_name, _ in subtask_rollouts if rol_num == rollout_number]:
            subtask_rollouts.append([rollout_number, 'FINISH', current_timestep])

        if line[0] == 'G':
            object_goal_distance = float(line[1])
            object_goal_distances.append([rollout_number, object_goal_distance, current_timestep])
        if line[0] == 'E':  
            end_effector_distance_to_object = float(line[1])
            end_effector_distances_to_object.append([rollout_number, end_effector_distance_to_object, current_timestep])
            
        
    # fix incorrect rollout numbers
    for rollouts in [subtask_rollouts, object_goal_distances, end_effector_distances_to_object]:
        prev_rollout_number = 0
        rollout_number = 0
        for rollout in rollouts:
            og_rollout_num = rollout[0]
            if rollout[0] != prev_rollout_number:
                rollout_number += 1
            rollout[0] = rollout_number
            prev_rollout_number = og_rollout_num
    

    #subtask_timesteps = {(rol_num, timestep) : subtask for rol_num, subtask, timestep in rollouts}
    subtask_timesteps = {(rol_num, subtask) : timestep for rol_num, subtask, timestep in subtask_rollouts}
    object_goal_distances = {(rol_num, timestep) : dist for rol_num, dist, timestep in object_goal_distances}
    end_effector_distances_to_object = {(rol_num, timestep) : dist for rol_num, dist, timestep in end_effector_distances_to_object}

    #print(subtask_timesteps)
    # print(object_goal_distances)
    return subtask_timesteps, object_goal_distances, end_effector_distances_to_object
    


def load_traj_file(file, pointer=8):
    """
    args: pointer - end position of the numeric data to extract from the file. You currently only 
    need to define this for the time_taken_file
    """
    rollouts = []
    rollout = []
    labels = []
    all_uncertainties = []
    all_timesteps = []
    print(f"Loading traj file at {file}")
    with open(file, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line = line.strip().split()
            timestep = int(line[0])
            uncertainties = [float(f) for f in line[1:pointer]]
            
            label = line[-1]

            if timestep == 0: 
                if rollout:
                    rollouts.append(rollout)
                    rollout = []
                labels.append(True if label == 'True' else False)

                if len(rollouts) >= args.num_rollouts:
                    break

            if timestep % 1 == 0:
                rollout.append((timestep, uncertainties))
                all_timesteps.append(timestep)
                all_uncertainties.append(uncertainties)

    # Add final rollout
    if rollout:
        rollouts.append(rollout)

    return rollouts, labels, all_uncertainties, all_timesteps



def plot_rollouts_uncertainties(rollouts, labels, results_path, parameters, graph_params,
                  subtask_timesteps, object_goal_distances, end_effector_distances_to_object, 
                  duration=800, joints_to_plot=[_ for _ in range(7)]):
    # define a window for each joint
    windows = {joint_num: deque(maxlen=parameters[joint_num]['window_size']) for joint_num in joints_to_plot} 
    # Store all scores and labels for each joint over all rollouts

    joint_all_scores = {joint_num: [] for joint_num in joints_to_plot}
    joint_all_labels = {joint_num: [] for joint_num in joints_to_plot}
    all_scores = []
    all_labels = []
    successful_rollout_means = {joint_num: [] for joint_num in joints_to_plot} 
    failed_rollout_means = {joint_num: [] for joint_num in joints_to_plot}
    num_joints = 7
    for i, rollout in enumerate(rollouts):
        if i % 50 == 0:
            print(f"Progress: {i} rollouts")
        rollout_timesteps = [t for t, _ in rollout]
        rollout_uncertainties = [unc for _, unc in rollout]

        # define a set of unsafe timesteps for the current rollout

        # transpose the list of uncertainties so each sublist is for one joint
        rollout_uncertainties_per_joint = list(zip(*rollout_uncertainties))

        for joint_num in range(num_joints):
            unsafe_timesteps = []

            fig, ax1 = plt.subplots(figsize=(10, 6))
            ax2 = ax1.twinx()
            ax2.set_ylabel('Object Goal Distance')
            # Plot uncertainties on the first y-axis
            ax1.set_ylim(0, graph_params[joint_num]['y_lim'])
            ax1.set_xlabel('Timestep')
            ax1.set_ylabel('Uncertainty Value')
            ax1.grid(True)

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

                plotted_line = ax1.plot(rollout_timesteps_to_plot, rollout_uncertainties_per_joint_to_plot, alpha=0.3, label=f"Joint {joint_num}")
                mean = np.mean(rollout_uncertainties_per_joint_to_plot)
                mean = [mean for _ in range(len(rollout_timesteps_to_plot))]
                unc_threshold_plot = [parameters[joint_num]['unc_threshold'] for _ in range(len(rollout_timesteps_to_plot))]

                # plot subtasks
                for subtask in ['APPR', 'GRASP', 'LIFT', 'FINISH']:
                    try:
                        rt = subtask_timesteps[(i, subtask)]
                        if rt < duration: 
                            ax1.scatter(rt, 0, zorder=4, label=f"{subtask} Completion")
                    except KeyError:
                        continue


                ax1.plot(rollout_timesteps_to_plot, mean, label=f"Joint {joint_num} mean uncertainty", linestyle='--', alpha=0.4, color=plotted_line[0].get_color()) 
                ax1.plot(rollout_timesteps_to_plot, unc_threshold_plot, label=f"Uncertainty Threshold", linestyle='--', alpha=0.4, color='red')
                # plot object to goal distances at each timestep
                # object_goal_distances_to_plot = [object_goal_distances.get((i, t-1), np.NaN) for t in rollout_timesteps_to_plot]
                # ax2.plot(rollout_timesteps_to_plot, object_goal_distances_to_plot, label=f"Object Goal Distance",  color='purple')
                # plot end effector to object distances at each timestep
                #end_effector_distances_to_object_to_plot = [end_effector_distances_to_object.get((i, t-1), np.NaN) for t in rollout_timesteps_to_plot]
                #ax2.plot(rollout_timesteps_to_plot, end_effector_distances_to_object_to_plot, label=f"EE Object Distance",  color='orange')
                
                if labels[i] == True:
                    successful_rollout_means[joint_num].append(mean)
                else:
                    failed_rollout_means[joint_num].append(mean)

                rolling_mean_uncs_list = []
                rolling_mean_uncs = 0
                # simulate the real-time window (uncertainty values come in one timestep at a time...)
                for timestep, unc in zip(rollout_timesteps_to_plot, rollout_uncertainties_per_joint_to_plot):
                    rolling_mean_uncs += unc / len(rollout_timesteps)
                    rolling_mean_uncs_list.append(rolling_mean_uncs)
                    windows[joint_num].append((timestep, unc))
                    unc_threshold = parameters[joint_num]['unc_threshold']
                    peaks = sum(1 for _, curr_unc in windows[joint_num] if curr_unc > unc_threshold)
                    if peaks > parameters[joint_num]['max_peaks']:
                        #print(f"number of peaks in window: {peaks} for rollout {i}")
                        # backtrack to get the timesteps for the entire window - so it says something like "there is danger in this window"
                        for unsafe_timestep, unsafe_uncertainty in windows[joint_num]:
                            unsafe_timesteps.append((unsafe_timestep, unsafe_uncertainty))
                #plt.plot(rollout_timesteps_to_plot, rolling_mean_uncs_list[:duration], label='Rolling Mean Uncertainty', linestyle='--', alpha=0.4)
                # plot the unsafe windows 
                # Extract x and y
                unsafe_x = [t for t, _ in unsafe_timesteps]
                unsafe_y = [u for _, u in unsafe_timesteps]

                # sort by timestep to ensure proper line order
                unsafe_sorted = sorted(zip(unsafe_x, unsafe_y))
                unsafe_x, unsafe_y = zip(*unsafe_sorted) if unsafe_sorted else ([], [])

                # with open(f"{results_path}/Joint{joint_num}/predicted_unsafe_timesteps_rollout_{i}.txt", 'w') as file:
                #     for ux in set(unsafe_x):
                #         file.write(f"{ux}\n")
                try:
                    with open(f"{results_path}/gold_standard_unsafe_timesteps_rollout_{i}.txt", 'r') as file:
                        true_unsafe_timesteps = file.readlines()
                        if len(true_unsafe_timesteps) > 0:
                            true_unsafe_timesteps_ranges = [
                                    (int(ut_min.strip()), int(ut_max.strip())) for ut_min, ut_max in 
                                    [ut.split() for ut in true_unsafe_timesteps]
                            ]

                            pred_unsafe_set = set(unsafe_x)
                            true_unsafe_set = set()
                            for ut_min, ut_max in true_unsafe_timesteps_ranges:
                                for true_timestep in range(ut_min, ut_max + 1):
                                    true_unsafe_set.add(true_timestep)
                            
                            total_rollout_timesteps_set = {t for t in range(len(rollout_timesteps))}
                            true_safe_set = total_rollout_timesteps_set - true_unsafe_set
                            pred_safe_set = total_rollout_timesteps_set - pred_unsafe_set
                            # # timesteps from pred set that are in true set
                            # TP = len(pred_unsafe_set & true_unsafe_set)
                            # # timesteps from pred set that are not in true set
                            # FP = len(pred_unsafe_set - true_unsafe_set)
                            # # timesteps from true set that are not in pred set (missing)
                            # FN = len(true_unsafe_set - pred_unsafe_set)
                            # # timesteps not in pred set and also not in true set
                            # TN = len(true_safe_set & pred_safe_set)
        
                            # precision = TP / (TP + FP) if (TP + FP) > 0 else 0
                            # recall    = TP / (TP + FN) if (TP + FN) > 0 else 0
                            # accuracy  = (TP + TN) / (TP + TN + FP + FN) if (TP + TN + FP + FN) > 0 else 0
                            
                            # TPR = TP / (TP + FN)
                            # FPR = FP / (FP + TN)

                            # print(f"Results for Joint {joint_num}, Rollout {i}")
                            # print(f"TP: {TP}, FN: {FN}, FP: {FP}, TN: {TN}")
                            # print(f"Precision: {precision:.2%}")
                            # print(f"Recall:    {recall:.2%}")
                            # print(f"TPR:       {TPR:.2%}")
                            # print(f"FPR:       {FPR:.2%}")
                            # print(f"Accuracy:  {accuracy:.2%}")

                            scores = rollout_uncertainties_per_joint[joint_num]
                            labels_per_ts = [1 if t in true_unsafe_set else 0 for t in rollout_timesteps]
                            joint_all_scores[joint_num].extend(scores)
                            joint_all_labels[joint_num].extend(labels_per_ts)

                            # fpr, tpr, thresholds = roc_curve(labels_per_ts, scores)
                            # roc_auc = auc(fpr, tpr)
                            
                            # title = f"ROC Curve for Joint {joint_num}, Rollout {i}"
                            # plt.figure(figsize=(8, 6))
                            # plt.plot(fpr, tpr, label=f'ROC (AUC = {roc_auc:.2f})')
                            # plt.plot([0, 1], [0, 1], linestyle='--', color='grey', label='Random')
                            # plt.xlabel('False Positive Rate')
                            # plt.ylabel('True Positive Rate')
                            # plt.title(title)
                            # plt.legend(loc='lower right')
                            # plt.grid(True)
                            # plt.tight_layout()
                            # plt.savefig(f"{results_path}/Joint{joint_num}/ROC_Rollout_{i}.png")
                            # plt.close()
                except FileNotFoundError as filenotfounderror:
                    with open(f"{results_path}/gold_standard_unsafe_timesteps_rollout_{i}.txt", 'w'):
                        pass 
                    os.chmod(f"{results_path}/gold_standard_unsafe_timesteps_rollout_{i}.txt", 0o666) # set write permissions because i only have sudo permissions inside the docker container
                
                # with open(f"{results_path}Joint{joint_num}/unsafe_timesteps.txt", 'w') as file:
                #     for ux in unsafe_x:
                #         file.write(f"{ux}\n") 
                ax1.plot(unsafe_x, unsafe_y, color='red', alpha=0.4, linewidth=1, linestyle='-', zorder=2, label=f"Detected Unsafe Window (Joint {joint_num})")
                
                # for window_size in [10]:
                #     # Plot average uncertainty over each window
                #     window_avg_timesteps = []
                #     window_avg_values = []
                #     # temp_window = deque(maxlen=parameters[window_size])
                #     temp_window = deque(maxlen=window_size)
                #     for timestep, unc in zip(rollout_timesteps_to_plot, rollout_uncertainties_per_joint_to_plot):
                #         temp_window.append((timestep, unc))
                #         # if len(temp_window) == parameters[window_size]:
                #         if len(temp_window) == window_size:
                #             # avg_unc = sum(u for _, u in temp_window) / parameters[window_size]
                #             avg_unc = sum(u for _, u in temp_window) / window_size
                #             end_timestep = temp_window[-1][0]
                #             window_avg_timesteps.append(end_timestep)
                #             window_avg_values.append(avg_unc)

                #     plt.plot(window_avg_timesteps, window_avg_values, zorder=3, label=f"Sliding Window Avg (Joint {joint_num}) for Window Size {window_size}")


                # Reset the original window buffer
                windows[joint_num].clear()

            title = f"Uncertainty for joint {joint_num} for: Rollout {i}, Model: {model_arch}, Task: {task}, {'Success' if labels[i] == True else 'Failure'}"
            # plt.title(title)
            # plt.xlabel('Timestep')
            # plt.ylabel('Uncertainty Value')
            fig.suptitle(title)
            lines, labels1 = ax1.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()
            ax1.legend(lines + lines2, labels1 + labels2, loc='upper right')
            
            # plt.legend()
            # plt.grid(True)
            # plt.tight_layout()
            #plt.savefig(f"{results_path}Joint{joint_num}/{title}.png")
            fig.savefig(f"{results_path}/Joint{joint_num}/{title}.png")
            plt.close()
   
    for joint_num in joints_to_plot:
        if len(set(joint_all_labels[joint_num])) < 2:
            print(f"Skipping Joint {joint_num} — only one class in labels.")
            continue

        fpr, tpr, thresholds = roc_curve(joint_all_labels[joint_num], joint_all_scores[joint_num])
        roc_auc = auc(fpr, tpr)

        plt.figure(figsize=(8, 6))
        plt.plot(fpr, tpr, label=f'ROC (AUC = {roc_auc:.2f})')
        plt.plot([0, 1], [0, 1], linestyle='--', color='grey', label='Random')
        plt.xlabel('False Positive Rate')
        plt.ylabel('True Positive Rate')
        plt.title(f'ROC Curve — Joint {joint_num} (All Rollouts)')
        plt.legend(loc='lower right')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(f"{results_path}/Joint{joint_num}/ROC_All_Rollouts.png")
        plt.close()

    for joint_num in range(num_joints):

        if joint_num in joints_to_plot:
            success_current_joint_means = [mean[joint_num] for mean in successful_rollout_means[joint_num]]
            failed_current_joint_means = [mean[joint_num] for mean in failed_rollout_means[joint_num]]
            
            current_joint_successful_mean = sum(success_current_joint_means) / len(success_current_joint_means) if len(success_current_joint_means) != 0 else 0
            current_joint_failed_mean = sum(failed_current_joint_means) / len(failed_current_joint_means) if len(failed_current_joint_means) != 0 else 0
            
            # mean_diff = 0.3  # try 0.01, 0.05, 0.1 for different overlaps (smaller means more overlap)
            # std_dev = 0.02
            # success_data = np.random.normal(loc=0.02, scale=std_dev, size=600)
            # # success_data =  np.clip(success_data, 0, 1)

            # fail_data = np.random.normal(loc=0.02 + mean_diff, scale=std_dev, size=400)
            # fail_data = np.clip(fail_data, 0, 1)
            
            success_data = success_current_joint_means
            fail_data = failed_current_joint_means
            #fail_data = [f + 0.001 for f in success_current_joint_means]
            
            # Estimate PDFs using KDE
            f_kde = gaussian_kde(fail_data)
            s_kde = gaussian_kde(success_data)

            # Evaluate over a range
            x_vals = np.linspace(0, graph_params[joint_num]['y_lim'], 1000)
            f_vals = f_kde(x_vals)
            s_vals = s_kde(x_vals)

            # Normalize
            f_vals /= np.trapz(f_vals, x_vals)
            s_vals /= np.trapz(s_vals, x_vals)


            # Compute Hellinger
            hellinger_distance_kde = (1 / np.sqrt(2)) * np.sqrt(np.trapz((np.sqrt(f_vals) - np.sqrt(s_vals))**2, x_vals))

            # Plot KDEs
            plt.figure(figsize=(10, 6))
            plt.plot(x_vals, s_vals, label=f"Success KDE\nHellinger distance: {'{:.3f}'.format(hellinger_distance_kde)}", color='green',
                 linewidth=2
            )
            plt.plot(x_vals, f_vals, label=f"Failure KDE", color='red', linewidth=2)

            # spline = CubicSpline(x_vals, f_vals)
            # y_dense = spline(x_vals)

            # first_derv = spline.derivative(nu=1)
            # second_derv = spline.derivative(nu=2)
            # stationary_points = first_derv.solve(y=0.0)
            # maxima_points = [sp for sp in stationary_points if second_derv(sp) < 0]
            # print(maxima_points)
            # plt.plot(x_vals, y_dense, label='Spline curve', linestyle='--', zorder=3)



            #Threshold to cut off the tail where density is negligible
            threshold = 1e-3  
            # Find max x where density is still above threshold for either curve
            mask = (f_vals > threshold) | (s_vals > threshold)
            x_max_dynamic = x_vals[mask][-1]  # last x where density is above threshold

            plt.xlim(0, x_max_dynamic)
            plt.title(f"KDE - Joint {joint_num}")
            plt.xlabel("Mean Uncertainty")
            plt.ylabel("Density")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(f"{results_path}/Joint{joint_num}/kde_joint_{joint_num}.png")
            plt.close()

            print("======================")
            print(f"Average mean over all successful rollouts for joint {joint_num}: {current_joint_successful_mean}")
            print(f"Average mean over all failed rollouts for joint {joint_num}: {current_joint_failed_mean}")
            #print(f"Hellinger's distance hist between successful and failed rollouts for joint {joint_num}: {hellinger_distance_hist}")
            print(f"Hellinger's distance kde between successful and failed rollouts for joint {joint_num}: {hellinger_distance_kde}")
            print("======================")
            
            
            plt.figure(figsize=(10, 6))
            # plt.ylim(bottom=0)
            plt.ylim(top=0.1)
        
            plt.bar(['Success', 'Failure'], [current_joint_successful_mean, current_joint_failed_mean], 
                    label=[f"{'{:.9f}'.format(current_joint_successful_mean)}", "{:.9f}".format(current_joint_failed_mean)], color=['green', 'red'])
            title = f"Average Mean Over All Successful Rollouts for Joint {joint_num}"
            plt.title(title)
            plt.xlabel('Task Result')
            plt.ylabel('Average Uncertainty Value')
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.savefig(f"{results_path}/Joint{joint_num}/{title}.png")
            plt.close()



def plot_trajectory(traj_info, results_path):
    for i, (rollout_actions, rollout_max_actions, rollout_min_actions) in enumerate(zip(traj_info['actions'], traj_info['max'], traj_info['min'])):
        timesteps = [t for t, _ in rollout_actions]
        actions = np.array([a for _, a in rollout_actions])
        max_actions = np.array([a for _, a in rollout_max_actions])
        min_actions = np.array([a for _, a in rollout_min_actions])

        #print(actions.shape, max_actions.shape, min_actions.shape)
        
        for joint_num in range(7):
        
            title = f"Trajectory for Rollout {i}, Joint {joint_num}"
            plt.figure(figsize=(10, 6))
            current_joint_actions = actions[:, joint_num]
            current_joint_max_actions = max_actions[:, joint_num]
            current_joint_min_actions = min_actions[:, joint_num]


            plt.fill_between(
                timesteps,
                current_joint_min_actions,
                current_joint_max_actions,
                color='blue',
                alpha=0.1,
                label='Action Range'
            )

            plt.plot(timesteps, current_joint_max_actions, label=f"Max Trajectory", color='blue', alpha=0.1)
            plt.plot(timesteps, current_joint_actions, label=f"Mean Trajectory", color='blue')
            plt.plot(timesteps, current_joint_min_actions, label=f"Min Trajectory",  color='blue', alpha=0.1)

            plt.tight_layout()
            plt.legend()
            plt.grid(True)
            plt.xlabel('Timestep')
            plt.ylabel('Joint Position')
            plt.savefig(f"{results_path}/Joint{joint_num}/{title}.png")
            plt.close()
        



# model_arch = "BC_RNN_GMM"
# task = "stack_cube" # stack_cube or pick_place
# model_name = f"model2"

# uncertainties_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/uncertainties.txt"


# uncertainties_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/video_plots/uncertainties.txt"
# results_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/Isaac-Stack-Cube-Franka-IK-Rel-v0/video_plots" 



parameters = {
        'stack_cube': {
             6: {
                'unc_threshold': 0.05,
                'max_peaks': 15,
                'window_size': 30,
                'max_uncertain_windows': 3
            },
            5: {
                'unc_threshold': 0.06,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 1
            },

            4: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            3: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            2: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            1: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            0: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            }
        },
        'pick_place': {
            6: {
                'unc_threshold': 0.05,
                'max_peaks': 15,
                'window_size': 30,
                'max_uncertain_windows': 3
            },
            5: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 1
            },

            4: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            3: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            2: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            1: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            0: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            }

        }
    }  

graph_params = {
    'stack_cube' : {
        6: {
            'y_lim' : 1.0
        },
        5 : {
            'y_lim': 0.1
        },
        4 :{
            'y_lim' : 0.1
        },
        3: {
            'y_lim' : 0.1
        },
        2: {
            'y_lim' : 0.1
        },
        1: {
            'y_lim' : 0.1
        },
        0: {
            'y_lim' : 0.1
        }
    },
    'pick_place' : {
        6: {
            'y_lim' : 1.0
        },
        5 : {
            'y_lim': 0.1
        },
        4 :{
            'y_lim' : 0.1
        },
        3: {
            'y_lim' : 0.1
        },
        2: {
            'y_lim' : 0.1
        },
        1: {
            'y_lim' : 0.1
        },
        0: {
            'y_lim' : 0.1,
            
        }
    }
}

parser = argparse.ArgumentParser()
parser.add_argument('--num_rollouts', type=int, default=999999, help='The number of rollouts to plot the uncertainties for.')

args = parser.parse_args()


model_arch = "BC_RNN_GMM"
task = "pick_place" # stack_cube or pick_place
model_name = f"ensemble"
number = 'experiment2'

results_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/run_{number}"

uncertainties_path = f"{results_path}/uncertainties{number}.txt"
actions_path = f"{results_path}/actions{number}.txt"
min_actions_path = f"{results_path}/min_actions{number}.txt"
max_actions_path = f"{results_path}/max_actions{number}.txt"
time_taken_path = f"{results_path}/time_taken{number}.txt"
rollout_log_path = f"{uncertainties_path[:len(uncertainties_path)-4]}_rollout_log.txt"

rollout_log_path = f"{uncertainties_path[:len(uncertainties_path)-4]}_rollout_log.txt" # remove the '.txt' and add rollout_log.txt

uncertainty_results_path = f"{results_path}/uncertainty_plots"
trajectory_results_path = f"{results_path}/trajectory_plots"
os.makedirs(uncertainty_results_path, exist_ok=True)
os.makedirs(trajectory_results_path, exist_ok=True)
# loghelper.namefile = rollout_log_path

# clear joint paths
for joint_num in range(7):
    uncertainty_plots_joint_num_dir_path = f"{uncertainty_results_path}/Joint{joint_num}"
    trajectory_plots_joint_num_dir_path = f"{trajectory_results_path}/Joint{joint_num}"
    os.makedirs(uncertainty_plots_joint_num_dir_path, exist_ok=True)
    os.makedirs(trajectory_plots_joint_num_dir_path, exist_ok=True)
    for dir_path in [uncertainty_plots_joint_num_dir_path, trajectory_plots_joint_num_dir_path]:
        for file_name in os.listdir(dir_path):
            os.remove(os.path.join(dir_path, file_name))
      

subtask_timesteps, object_goal_distances, end_effector_distances_to_object  = process_rollout_logger(rollout_log_path)

rollouts_uncertainties, labels, all_uncertainties, all_timesteps = load_traj_file(uncertainties_path)
rollouts_actions, _, _, _ = load_traj_file(actions_path)
rollouts_max_actions, _, _, _ = load_traj_file(max_actions_path)
rollouts_min_actions, _, _, _ = load_traj_file(min_actions_path)
rollouts_times_taken, _, _, _ = load_traj_file(time_taken_path, pointer=2)

traj_info = {
    'actions': rollouts_actions,
    'max': rollouts_max_actions,
    'min': rollouts_min_actions,
    'time_taken': rollouts_times_taken
}


plot_rollouts_uncertainties(rollouts_uncertainties, labels, uncertainty_results_path,
              parameters[task], graph_params[task], 
              subtask_timesteps, object_goal_distances, end_effector_distances_to_object,
              duration=800, joints_to_plot=[0, 1, 2, 3, 4, 5, 6]
            )


plot_trajectory(traj_info, trajectory_results_path)


# Hellinger distance data for each policy and joint
# hellinger_data = {
#     "minstd": {
#         "joint 0": 0.9515545164079814,
#         "joint 1": 0.9621043249318753,
#         "joint 2": 0.9527533890746112,
#         "joint 3": 0.9429505059792747,
#         "joint 4": 0.9650761111142897,
#         "joint 5": 0.9493641817087877,
#         "joint 6": 0.9219050370756138,
#     },
#     "median": {
#         "joint 0": 0.9254888613310024,
#         "joint 1": 0.9318249066044189,
#         "joint 2": 0.9454337442794406,
#         "joint 3": 0.9507902695862722,
#         "joint 4": 0.9465258444879787,
#         "joint 5": 0.9497317568066842,
#         "joint 6": 0.559237739819455,
#     },
#     "mean": {
#         "joint 0": 0.883771399345651,
#         "joint 1": 0.9250691897193906,
#         "joint 2": 0.92959787115467,
#         "joint 3": 0.9243240153860592,
#         "joint 4": 0.9499016758179112,
#         "joint 5": 0.9562868227795528,
#         "joint 6": 0.8730400182033042,
#     }
# }

# joints = list(hellinger_data["minstd"].keys())
# x = range(len(joints))

# # Extract values for each model
# minstd_vals = list(hellinger_data["minstd"].values())
# median_vals = list(hellinger_data["median"].values())
# mean_vals = list(hellinger_data["mean"].values())

# bar_width = 0.25

# plt.figure(figsize=(10, 6))

# plt.bar([i - bar_width for i in x], minstd_vals, width=bar_width, label='minstd', color='mediumpurple')
# plt.bar(x, median_vals, width=bar_width, label='median', color='lightgreen')
# plt.bar([i + bar_width for i in x], mean_vals, width=bar_width, label='mean', color='cadetblue')

# plt.xticks(x, joints)
# plt.ylabel('Hellinger Distance')
# plt.title('Hellinger Distance per Joint across Policy Types')
# plt.legend(loc='lower left')
# plt.tight_layout()
# plt.grid(True, axis='y', linestyle='--', alpha=0.7)

# plt.savefig(f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/HD_plot.png")


