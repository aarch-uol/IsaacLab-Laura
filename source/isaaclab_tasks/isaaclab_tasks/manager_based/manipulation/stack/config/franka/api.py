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
    # def __init__(self, model_name="codestral:22b"):
    #     self.model_name = model_name

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
    The content of the output file should only differ by the objects spawned from the given file.

    ### Task description

    User description: {user_input}
    Think about which laboratory equipment from the original file would be used to complete the task in the user description.
    Look at the user description and decide how many tasks have been described, write the number of tasks in a comment under the imports.

    ### Imports

    - **`PourEnvCfg` is in `lab_env_cfg` file in `stack` folder**.
    - If `pour` in user description, then change import StackEnvCfg to PourEnvCfg.

    ### DoneTerm Definitions

    - Write **exactly** 4 DoneTerms for:
        - Two terms for object dropping as shown in the given file, one for object1 and one for object2.
        - Must include one of (Success Term): mdp.objects_stacked function or mdp.object_reached_goal.
        - A time out term, the DoneTerm includes: func=mdp.time_out, time_out=True.
    Strictly Ensure that there are 4 DoneTerms under TerminationsCfg.

    Success Condition Selection Rules:
        - For objects_stacked (used for any tasks involving stacking object1 onto object2):
            - Use objects_stacked **only** if object_stacked is appropriate for the final subtask.
            - If there is one task: objects_stacked only param `object_2_cfg`: SceneEntityCfg object2.
            - If there is two tasks: objects_stacked only param `object_2_cfg`: SceneEntityCfg object3.
        - For object_reached_goal (used if stacking isn't applicable for final subtask):
            - Use object_reached_goal **only** if object_near_goal is appropriate for the final subtask.
            - Requires no params

    ### ObsTerm Definitions

    - Create an ObservationsCfg Class.
    - Within the ObservationsCfg Class, create two Classes: PolicyCfg and SubtasksCfg, inheriting from ObsGroup.
    
    Inside the PolicyCfg, define ObsTerms with **only** the following **9** functions and no params:
        IF THERE IS ONLY ONE TASK (`func=`):
            - `last_action`, `joint_pos_rel`, `joint_vel_rel`, `object_obs`, `object_positions_in_world_frame`, `object_orientations_in_world_frame`, `ee_frame_pos`, `ee_frame_quat`, `gripper_pos`
        IF THERE ARE TWO TASKS (`func=`):
            - `last_action`, `joint_pos_rel`, `joint_vel_rel`, `object_obs2`, `object_positions_in_world_frame2`, `object_orientations_in_world_frame2`, `ee_frame_pos`, `ee_frame_quat`, `gripper_pos`

    SUBTASK GENERATION RULES:
    Task Context:
        In SubtasksCfg, create a logical sequence of subtasks using ObsTerm for a robot to complete the user-defined task(s).
        - Single Task → Subtasks use object1 as the main object, object2 as apparatus.
        - Two Tasks → Generate two sequences:
            Task 1: object1 (main), object2 (apparatus)
            Task 2: object1 (main), object3 (apparatus)
        - The sequence of task 1 must be fully completed before starting the sequence for task 2.

    Available Subtask Functions (func=):
        - Core Subtasks: reach_object, object_grasped, is_object_lifted, object_reached_midgoal, reach_object2
        - Optional Subtasks: pour_object, reorient_object (follows pour_object if used)
        - Final Termination Subtask must include either (ObsTerm(func=)): `object_stacked`, `object_near_goal`.
        - Use reach_object2 for a second reach in a task.

    SUBTASK FUNCTION PARAMETERS:
        - `reach_object` / `reach_object2` requires: `ee_frame_cfg`, `object_cfg`, `threshold`.
        - `object_grasped` requires: `ee_frame_cfg`, `robot_cfg`, `object_cfg`.
        - `is_object_lifted` requires: `threshold`, `object_cfg`.
        - `object_reached_midgoal` requires: `threshold`, `command_name = "object_pose"`.
        - `pour_object` / `reorient_object` requires: `angle_threshold = 45 / 175`, `ee_frame_cfg`.

    Termination Function Rules:
        - For `object_stacked` (used for tasks involving placing object1 onto apparatus):
            - Use `object_stacked` **only** if appropriate for the final subtask.
            - Requires params:
                - `robot_cfg` → SceneEntityCfg("robot")
                - `upper_object_cfg` → SceneEntityCfg("object1")
                - `lower_object_cfg`:
                    - IF ONE TASK → SceneEntityCfg("object2")
                    - IF TWO TASKS → SceneEntityCfg("object3")
        - For `object_near_goal` (used if stacking isn't applicable for final subtask):
            - Requires:
                - `threshold`
                - `robot_cfg` → SceneEntityCfg("robot")
                - `object_cfg` → SceneEntityCfg("object1")
        - **The termination function must be the final subtask for each sequence of subtasks**.

    General Parameter Mapping Rules:
        - Use SceneEntityCfg("object1") for any subtask with `"object"` in the name.
        - IF ONE TASK:
            - Use SceneEntityCfg("object2") for `"object2"` subtasks.
        - IF TWO TASKS:
            - For Task 1: use SceneEntityCfg("object2") for `"object2"` subtasks.
            - For Task 2: use SceneEntityCfg("object3") for `"object2"` subtasks.
        - Use SceneEntityCfg("ee_frame") for all `ee_frame_cfg` parameters.
        - Use `threshold` values between `0.05` and `0.1`.

    At the end of ObservationsCfg Class create an instance for: PolicyCfg and SubtasksCfg each with `enable_corruption`, `concatenate_terms` which are both `False`.
    Note: All ObsTerm functions are sourced from the `mdp` folder.

    ### FrankaCubeStack Class Definitions

    Use the `@configclass` decorator for any Classes, as in the given file.

    Change the inheritance of `FrankaCubeStackEnvCfg(StackEnvCfg)` only IF the task involves `pour` in the task description:
        - Change `StackEnvCfg` to `PourEnvCfg`.
    If `pour` is not in the task description then keep `StackEnvCfg`.

    ### Task Specific Objects

    - Use **only** objects defined in the original file — do **not invent new ones**.
    - You are **strictly forbidden** from instantiating **all** glassware or lab equipment.
    - Instantiate only the **Franka robot** and the **objects required** to complete the user-defined task.
    - **Do not use all of the objects from the original file**.
    - The **hot plate** object in the USD file refers to a **magnetic stir plate**.
    - The “main object” is the one physically moved within the task. You must decide which one that is based on the task description.
    - This main object will be a piece of glassware.
    - **Only include the "main object" for the first task**.

    In the output file:
        ***CRITICAL NAMING RULE — FOLLOW EXACTLY*** 
        For the task's **main object**, use this naming style:

        DO NOT USE THIS (placeholder style — REMOVE IT):  
        ```python
        self.scene.<glassware> = glassware.<glassware>
        ```
        INSTEAD, RENAME LIKE THIS (example if object1):
        ```python
        self.scene.object1 = glassware.<glassware>
        ```
        Do not include any placeholder code like <glassware> in your final output.
    - Replace <glassware> with the actual object name from the task, and assign it to a standardized scene name like object1.

    IMPORTANT: Do NOT treat all equipment the same. Follow the rules based on the number of tasks in the user description.
    CRITICAL: Task count = number of tasks described in user description.
    TASK COUNT CHECK:
        IF THERE IS ONLY ONE TASK:
            - Identify the lab equipment that the main object interacts with (this is the target or tool).
            - Instantiate that equipment and assign it the name object2.
        IF THERE ARE TWO TASKS:
            - Identify the lab equipment of Task 1 (the lab equipment performing the task or is the target of the task).
                - Rename the target object2 using the naming convention self.scene.object2 = glassware.<glassware>.
            - Identify the lab equipment of Task 2 (the lab equipment performing the task or is the target of the task).
                - Rename the target object3 using the naming convention self.scene.object3 = glassware.<glassware>.
            IMPORTANT: Both object2 and object3 **must** be in the output file.

    - If there are two names for an instance, keep only the name involving object1.
    - Each instantiated object must have a side comment explaining:
        - Whether it is the main object.
        - Which task it is used in.

    DO NOT:
    - Do not use object2 for both tasks.
    - Do not use object3 if there's only one task.
    - Do not name the target equipment anything other than object2 or object3 depending on task order.

    REMEMBER:
    - object1 = main object (the glassware being moved) — always the same, used for all tasks.
    - object2, object3 = equipment the main object interacts with (assigned by task number).
    - Count your instantiated objects. There must be no more than 3 total.

    Given/Original file to reference: {file_content}
    ```plaintext
    REMINDER: ALL object assignments **must use the format `self.scene.object1 = ...`** and **must not include placeholder code like `<glassware>`**.
    REMINDER: Do not use objects_stacked if both objects are **glassware**.

    """

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
