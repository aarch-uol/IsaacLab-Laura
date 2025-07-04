from openai import OpenAI
import re
import subprocess

# Create the client to talk to Ollama running locally
client = OpenAI(base_url="http://localhost:11434/v1", api_key="ollama")

class LLMClient:
    def __init__(self, model_name="qwen3:8b"):
        self.model_name = model_name
    # def __init__(self, model_name="gemma3:1b"):
    #     self.model_name = model_name
    # def __init__(self, model_name="gemma3:12b"):
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
    
    def user_description(self):
        # Could be within the output file function?
        user_input = input("Please give a description of the environment")
        return

    def output_file(self, file_path: str, output_file: str):
        """Reads a given file, connects to LLM through API and python code part of comment is written into a new file."""
        file_content = self.read_file(file_path)

        prompt = f"""
    Here is the content of the file:

    {file_content}

    This file defines a configuration class for part of a simulation environment.
    I am working with Isaaclab.
    I want you to output a code script.
    Write out the imports that are within the given file only once, at the **vert top** of your script.
    All of the import statements should start with "isaaclab".
    Do not repeat any imports elsewhere in the script.
    Do not invent, modify, or add new imports.

    Write of the first three event configurations using EventTerm() from the given file in the output file. 
    Within the EventTerm(), write the same params in the output file as in the given file.

    Your output script should include the robot and the two objects: a cube and a hot plate.
    Change the `name` of the cube object, for example the output should be self.scene.object1 = glassware.cube rather than self.scene.cube = glassware.cube
    Use @configclass for FrankaCubeStackEnvCfg() and EventsCfg() as in the given file.
    Define these objects in the class FrankaCubeStackEnvCfg(StackEnvCfg), and make sure the method super().__post_init__() is included in __post_init__().
    Write the method to Set Franka as the robot from the given file in the output file including the same semantic tags.
    Do not change the semantics for the plane and table in the FrankaCubeStackEnvCfg class.
    Other than the code relating to spawning objects, within the FrankaCubeStackEnvCfg class, all the code should be the same from the given file.

    Set the actions for the franka robot in the output file, the same as in the given file.
    Write the exact same code for the end effector frame transformations from in the output file as is in the given file.

    Specifically, ensure that:
    - The `FrameTransformerCfg` uses the  same `prim_path`, `visualizer_cfg`, and `debug_vis` settings.
    - The `target_frames` list contains `FrameTransformerCfg.FrameCfg` entries for all of: `panda_hand`, `panda_leftfinger`, and `panda_rightfinger`.
    - All field values (e.g., `prim_path`, `name`, `offset`) must exactly match the original file. Do not modify or invent new values.
    - Keep OffsetCfg property pos for the `panda_hand`, `panda_leftfinger`, and `panda_rightfinger` the same as in the given file.

    Strictly ensure:
    - No repeated imports.
    - No duplicated sections of more than 5 lines in the output.
    - Only spawn objects that have been mentioned in this prompt.
    - Output a single, complete Python file.
    - No SyntaxError in the required transforms code.
    """

    # {user_input} is a message input from the user
    # First look at the objects within the message input, these should be the only objects within the output script
    ## If message includes any information about the size, position or rotation of objects
    ## If message includes a task - task of stacking is possible, if another then ??? so far

    # Could give scales known for the different sizes of chemical equipment, then whatever size that is asked the known scale would be known.
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

        print(f"Extracted code saved to: {output_file}")
        return code

    
    def run_llm_file(self):
        terminal_code = [
            "./isaaclab.sh",
            "-p",
            "scripts/environments/teleoperation/teleop_se3_agent.py",
            "--task",
            "Isaac-Stack-LLM-Franka-IK-Rel-v0",
            "--num_envs",
            "1",
            "--teleop_device",
            "keyboard"
            ]
        
        terminal_result = subprocess.run(terminal_code)
        return terminal_result


def main():
    file_name = "source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/stack/config/franka/stack_joint_pos_env_cfg.py"
    output_file = "source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/stack/config/franka/generated_llm.py"
    model_client = LLMClient()
    code = model_client.output_file(file_name, output_file)
    print("\nExtracted Code:\n")
    print(code)
    model_client.run_llm_file()



if __name__ == "__main__":
    main()
