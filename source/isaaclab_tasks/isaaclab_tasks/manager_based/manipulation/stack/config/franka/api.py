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
    Here is the content of the file:

    {file_content}

    This prompt applies to IsaacLab-based simulation code. 
    Your task is to generate a single, valid Python file with no syntax errors. 
    The output should be minimal, correct, and closely match the structure and content of the given file.
    The content of the output file should only differ by the objects spawned from the given file.
    This file defines a configuration class for part of a simulation environment.

    ### Task description

    User description: {user_input}
    Think about what chemical equipment might be used to complete the task in the user description.

    ### Imports

    - Extract **only and all of the 11 lines of import statements** that are in the **given file**.
    - Add them **once at the very top** of your output script.
    - Every import must start with `isaaclab` or `glassware_files`.
    - **Do not repeat or invent any import**.
    - **Do not add extra imports**, even if they seem necessary.

    ### EventTerm Definitions

    - Include exactly the **first four EventTerm()** entry from the given file.
    - These must be written **exactly as-is**: same function, mode, and params.
    - Do not add, remove, or alter any parameters inside `params`.

    ### Class Definitions

    Use the `@configclass` decorator for both classes, as in the given file:
    - `FrankaCubeStackEnvCfg(StackEnvCfg)`
    - `EventsCfg(EventGroupCfg)`

    Change the inheritance of `FrankaCubeStackEnvCfg` if the task involves `pouring` in the task description:
    - Change `StackEnvCfg` to `PourEnvCfg`.
    If `pouring` is not in the task description then keep `StackEnvCfg`.

    In `FrankaCubeStackEnvCfg`:
    - Include a method called `__post_init__()` that contains `super().__post_init__()`.
    - **Do not change** the semantic tags for the plane or table.
    - **Do not modify** how robot actions or robot commands are set.
    - All code in this class must exactly match the given file — except for object spawning.

    ### Task Specific Objects

    - Use **only** objects defined in the original file — do **not invent new ones**.
    - Instantiate only the **Franka robot** and **objects required** to complete the user task.
    - The **hot plate** object in the USD file refers to a **magnetic stir plate**.
    - If needed, multiple instances of the same object are allowed.

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
    - The “main object” is the one moved within the task. You must decide which one that is based on the task description.
    - Rename the chosen main object to follow the self.scene.object1 naming convention as shown above.
    - Rename the lab equipment that is completing the task to follow the self.scene.object2 naming convention similar to above.
    - This main object will be a piece of glassware.
    - If there are two names for an instance, keep only the name involving object1.
    - Add a side comment indicating which object was chosen as the main object for the task.
    - Include as many objects within the scene as are expected for the task.

    ### Robot and Frame Transformations

    - Use the same robot setup code from the given file (Franka, with the correct semantic tags).
    - Add one FrameTransformerCfg with the following three FrameCfg entries in this exact order:
        1. panda_hand
        2. panda_leftfinger
        3. panda_rightfinger
    - For each FrameCfg:
    - Use the same prim_path, name, and OffsetCfg(pos=...) values as the original file.
    - Do not change or invent any values or fields.

    Strictly ensure:
    - I need a **single Python file** that:
    - 1. **Imports only once** from `glassware_files` (no repeated imports).
    - 2. **Spawn object needed for task only**
    - 3. **Include three `FrameTransformerCfg`, each with a FrameCfg entries in the target_frames list**
    - 4. **No syntax errors** — all functions and object definitions must be valid.
    - 5. **No extra code** — only the minimal necessary to fulfill the above.
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
