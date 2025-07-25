from enum import Enum
class LogType(Enum):
    APPR_OBJ1 = 0
    GRASP_OBJ1 = 1
    LIFT = 2
    MIDGOAL = 3
    APPR_OBJ2 = 4
    STACK = 5
    FINISH = 6

class ErrorType(Enum):
    TIMEOUT = 0
    JOINT_VIO = 1
    VEL_VIO = 2
    EFF_VIO = 3
    DROP = 4
    RESAMPLE = 5
    CONTACT = 6
    ORIENTATION = 7


class LoggingHelper:
    def __init__(self, logname : str= "docs/vialrack_4k.txt"):
        self.namefile = logname
    
    def startEpoch(self, epochnum : int):
        with open(self.namefile, "a") as f:
            f.write(f"S:{epochnum}:Starting \n")

    def stopEpoch(self, epochnum: int):
        with open(self.namefile, "a") as f:
            f.write(f"F:{epochnum}:Finished \n")

    def logsubtask(self, logtype : LogType):
        with open(self.namefile, "a") as f:
            f.write(f"{logtype.value}:{logtype.name}:TRUE \n")
    
    def logerror(self, errortype: ErrorType):
        with open(self.namefile, "a") as f:
            f.write(f"T{errortype.value}:{errortype.name}:TRUE \n")