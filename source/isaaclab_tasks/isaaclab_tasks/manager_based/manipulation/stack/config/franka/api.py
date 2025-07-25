from openai import OpenAI
import re
import subprocess

# Create the client to talk to Ollama running locally
client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")

class LLMClient:
    # def __init__(self, model_name="qwen3:8b"):
    #     self.model_name = model_name
    # def __init__(self, model_name="gemma3:1b"):
    #     self.model_name = model_name
    # def __init__(self, model_name="gemma3:12b"):
    #     self.model_name = model_name
    # def __init__(self, model_name="dolphin3:8b"):
    #     self.model_name = model_name
    def __init__(self, model_name="devstral:24b"):
        self.model_name = model_name

    def read_file(self, file_path: str):
        """Reads a file."""
        with open(file_path, "r") as f:
            return f.read()

    def extract_code(self, text: str) -> str:
        """Extracts the Python code block (```python ...```) from the LLM code."""
        # Extract the first Python code block (```python ... ```)
        code_blocks = re.findall(r"```(?:python)?\n(.*?)```", text, re.DOTALL)
        return code_blocks[0] if code_blocks else text  # fallback to full text

    def output_file(self, file_path: str, output_file: str):
        """Reads a given file, connects to LLM through API and python code part of comment is written into a new file."""

        user_input = input("Please give a description of the environment: ")
        file_content = self.read_file(file_path)

        prompt = f"""

    This prompt applies to IsaacLab-based simulation code. 
    Your task is to generate a single, valid Python file with no syntax errors. 
    The output should be minimal, correct, and closely match the structure and content of the given file.
    The content of the output file should only differ by the objects spawned from the given file.
    This file defines a configuration class for part of a simulation environment.

    ### Task description

    User description: {user_input}
    Think about what chemical equipment might be used to complete the task in the User description - use for ### Task Specific Objects.

    ### Imports

    - Extract **only and all of the 12 lines of import statements** that are in the **given file**.
    - Add them **once at the very top** of your output script.
    - Every import must start with `isaaclab` or `glassware_files`.
    - **Do not repeat or invent any import**.
    - **Do not add extra imports**, even if they seem necessary.
    - If `pour` in user description, then change import StackEnvCfg to PourEnvCfg.

    ### DoneTerm Definitions

    - Write **exactly** 5 DoneTerms for:
        - Two terms for object dropping as shown in the given file, one for object1 and one for object2.
        - For object1 also include an object dropping term for onto the table rather than below the table which is at 0.005 height.
        - A success term from mdp.objects_stacked function with no params term.
        - A time out term, the DoneTerm includes func=mdp.time_out and time_out=True.
    Strictly Ensure that there are 5 DoneTerms under TerminationsCfg.

    ### ObsGroup/ObsTerm Definitions

    - Create an ObservationsCfg Class.
    - Within the ObservationsCfg Class, create two Classes: PolicyCfg and SubtasksCfg, inheriting from ObsGroup.
    - At the end of ObservationsCfg Class create an instance for: PolicyCfg and SubtasksCfg.
    - Inside the PolicyCfg, define ObsTerms for only the following **9** observation functions: `last_action`, `joint_pos_rel`, `joint_vel_rel`, `object_obs`, 
    `object_positions_in_world_frame`, `object_orientations_in_world_frame`, `ee_frame_pos`, `ee_frame_quat`, `gripper_pos`.

    - In SubtasksCfg, create subtasks that logically sequence to complete the user-described task.
    - The subtasks should link and flow together to be checkpoints to complete the task description.
    - Use only the following func= values for the ObsTerm subtasks: reach_object, object_grasped, is_object_lifted, object_reached_midgoal, reach_object2, object_stacked,
    pour_object, reorient_object, object_reached_goal.
    - Use func=reach_object2 instead of func=reach_object if a second reach is needed.

    - The function `object_grasped` should have three params with the names: `ee_frame_cfg`, `robot_cfg` and `object_cfg`.
    - The function `reach_object`/`reach_object2` should have three params with the names: `ee_frame_cfg`, `object_cfg` and `threshold`.
    - The function `object_reached_goal`/`object_reached_midgoal` should have two params with the names `threshold` and `command_name`.
    - The function `is_object_lifted` should have two params with the names `threshold` and `object_cfg`.
    - The functions `pour_object`/`reorient_object` should have two params with the names `angle_threshold` = 45/175 (respectively) and `ee_frame_cfg`.

    Termination Function Rules:
        - For object_stacked (used for final stacking tasks):
            - Use object_stacked only if appropriate for the final subtask.
            - Requires params:
                - upper_object_cfg → SceneEntity "object1"
                - lower_object_cfg → SceneEntity "object2"
                - robot_cfg → SceneEntity "robot"
        - For object_reached_goal (used for final subtask if stacking isn't applicable):
            - Requires:
                - threshold
                - command_name: "object_pose"

    General Parameter Mapping Rules:
    - Use SceneEntity "object1" if the subtask name includes "object".
    - Use SceneEntity "object2" if the subtask name includes "object2".
    - Use SceneEntity "robot" only for object_grasped and object_stacked.
    - Use SceneEntity "ee_frame" for any ee_frame_cfg parameter.
    - The threshold value should be between 0.05 and 0.1.

    - All of the functions for the ObsTerms and are from the folder mdp.
    - For each of the `PolicyCfg` and `SubtasksCfg` Classes, create `post_init` with only `enable_corruption` and `concatenate_terms` which are both `False`.

    ### FrankaCubeStack Class Definitions

    Use the `@configclass` decorator for any Classes, as in the given file, including `FrankaCubeStackEnvCfg(StackEnvCfg)`

    Change the inheritance of `FrankaCubeStackEnvCfg` only if the task involves `pour` in the task description:
        - Change `StackEnvCfg` to `PourEnvCfg`.
    If `pour` is not in the task description then keep `StackEnvCfg`.

    In `FrankaCubeStackEnvCfg`:
        - Include a method called `__post_init__()` that contains `super().__post_init__()`.
        - Within `__post_init__` create an instance of ObservationsCfg() under # Set Observations.
        - **Do not change** the semantic tags for the plane or table.
        - Include **only and exactly all** of the `actions` and `commands` from the given file.
        - Use the same robot setup code from the provided file (Franka, with the correct semantic tags).

    ### Task Specific Objects

    - Use **only** objects defined in the original file — do **not invent new ones**.
    - Instantiate **only** the **Franka robot** and **objects required** to complete the user-defined task.
    - The **hot plate** object in the USD file refers to a **magnetic stir plate**.
    - The “main object” is the one moved within the task. You must decide which one that is based on the task description.
    - This main object will be a piece of glassware.

    In the output file:
    - For the task's main object, use this naming style:
        Replace (this code should not be present in output code): # Example for a <glassware>
            ```python
            self.scene.<glassware> = glassware.<glassware>
            ```
        with this:
            ```python
            self.scene.object1 = glassware.<glassware>
            ```
    - Rename the lab equipment that is completing the task or is the target to follow the self.scene.object2 naming convention similar to above.
    - If there are two names for an instance, keep only the name involving object1.
    - Add a side comment indicating which object was chosen as the main object for the task.
    - Include as many objects within the scene as are expected for the task.

    ### Robot and Frame Transformations

    - Write the `Frame Transformations` from the given file **exactly** at the end of the code.
    - Do not modify any of the values or change the structure from that block of code.
    - Do not invent, change, or omit any fields or values.
    - From that block, one FrameTransformerCfg, with the name `self.scene.ee_frame`, should be created with FrameCfgs for:
        1. panda_hand
        2. panda_leftfinger
        3. panda_rightfinger
    - With the names (respectively):
        1. end_effector
        2. tool_leftfinger
        3. tool_rightfinger
    - For each FrameCfg use **the same** values from the original code for:
        - prim_path
        - name
        - offset=OffsetCfg(pos=...)
    - The `FrameTransformerCfg` uses the  same `prim_path`, `visualizer_cfg`, `debug_vis` and `target_frames` values as the given file.
    - Keep OffsetCfg property `pos=` for the `panda_hand`, `panda_leftfinger`, and `panda_rightfinger` the same as in the given file.

    Given/Original file to reference: {file_content}

    """

    # {user_input} is a message input from the user
    # First look at the objects within the message input, these should be the only objects within the output script
    ## If message includes any information about the size, position or rotation of objects
    ## If message includes a task - task of stacking is possible, if another then ??? so far

    # Could give scales known for the different sizes of chemical equipment, then whatever size that is asked the known scale would be known.
    # Could create different sizes for chemical equipment which could be called to
    # Or to be less repetitive, it could know the general sizes of the equipment, allocate the sizes depending on how many there are
    # and then give the size of the respective one that was asked.

        response = client.chat.completions.create(
            temperature=0,
            model=self.model_name,
            stream=True,
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": prompt},
            ],
        )

        raw_response = ""
        for chunk in response:
            delta = chunk.choices[0].delta
            if delta.content:
                print(delta.content, end="", flush=True)
                raw_response += delta.content

        print("\n\nFull response received.\n")
        code = self.extract_code(raw_response)

        with open(output_file, "w") as f:
            f.write(code)

        if "weigh" in user_input.lower():
            state_machine = "weigh_lab_sm"
        elif "pour" in user_input.lower():
            state_machine = "pour_lab_sm"
        else:
            state_machine = "stack_lab_sm"

        print(f"Extracted code saved to: {output_file}")

        terminal_code = [
            f"./isaaclab.sh",
            f"-p",
            f"scripts/environments/state_machine/{state_machine}.py",
            f"--num_envs",
            f"1",
            ]
        # Run another .py file automatically
        terminal_result = subprocess.run(terminal_code)

        return code


def main():
    file_name = "source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/stack/config/franka/stack_joint_pos_env_cfg.py"
    output_file = "source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/stack/config/franka/generated_llm.py"
    model_client = LLMClient()
    code = model_client.output_file(file_name, output_file)



if __name__ == "__main__":
    main()
