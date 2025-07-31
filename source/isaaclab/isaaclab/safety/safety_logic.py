import math
import numpy
import torch 

    # create an instance of the safety logic class
    # obst: task space location of the obstacle
    # obst_rad : radius of the no-go around the obstacle
    # obst_uncert  : uncertainty associated with the obsacle [0 : 1]
class SafetyLogic:
    def __init__(self, obst : torch.tensor, obst_rad : float = 0.1, obst_uncert : float = 0.5, var_thresh : float = 0.6):
        self.obst = obst.to('cpu').numpy()
        self.obst_uncert: float = obst_uncert
        self.obst_rad: float = obst_rad 
        self.var_thresh : float = var_thresh
    
    # check for expected collision
    def exp_static_coll (self, eef_pos : torch.tensor)-> tuple:
        # returns true if the end effector is in a danger zone
        # (x−cx)2+(y−cy)2+(z−cz)2<r2.
        eef  = eef_pos.to('cpu').numpy()[0]
        obs  = self.obst[0]
        test = 0
        for i in range(len(eef)):
            test += (eef[i] - obs[i])**2
         # account for uncertainty 
     #   print("Safety logic : ", (test < (self.obst_rad * (1+self.obst_uncert))**2))
        return  test < (self.obst_rad**2), test
    
    def exp_variance(self, pol, obs) -> bool:
        var_dict = self._MC_dropout_uncertainty(pol, obs)
        if var_dict['variance'] > self.var_thresh:
            print("Uncertainty too high : ", var_dict['variance'])
        return var_dict['variance'] > 0.6 
    
    def _MC_dropout_uncertainty(self,policy, obs, niters=50):
        actions = [torch.from_numpy(policy(obs)) for i in range(niters)]
        actions = torch.stack(actions)
        
        mean = torch.mean(actions, dim=0)
        std = torch.std(actions, dim=0)
        variance = torch.sqrt(std)
        
        return {
            'mean': mean,
            'std': std,
            'variance': variance
        }

    #uncertainty_dict = MC_dropout_uncertainty(policy=policy, obs=obs, niters=15)
    #    print("Action variance : ", uncertainty_dict['variance'])