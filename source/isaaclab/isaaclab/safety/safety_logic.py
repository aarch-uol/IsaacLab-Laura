import math
import torch 

    # create an instance of the safety logic class
    # obst: task space location of the obstacle
    # obst_rad : radius of the no-go around the obstacle
    # obst_uncert  : uncertainty associated with the obsacle [0 : 1]
class SafetyLogic:
    def __init__(self, obst : torch.tensor, obst_rad : float = 0.1, obst_uncert : float = 0.5):
        self.obst = obst
        self.obst_uncert = obst_uncert
        self.obst_rad = obst_rad 
    
    def exp_coll (self, eef_pos : torch.tensor):
        # returns true if the end effector is in a danger zone
        return False