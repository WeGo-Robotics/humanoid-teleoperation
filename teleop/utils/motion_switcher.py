# for motion switcher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
# for loco client
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
import time

# MotionSwitcher used to switch mode between debug mode and ai mode
class MotionSwitcher:
    def __init__(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(1.0)
        self.msc.Init()

    def Enter_Debug_Mode(self):
        try:
            status, result = self.msc.CheckMode()
            while result['name']:
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
            return status, result
        except Exception as e:
            return None, None
    
    def Exit_Debug_Mode(self, retries=10, wait=0.5):
        # SelectMode('ai') can return 7002 right after leaving debug even when it
        # takes effect a moment later; retry and judge success by CheckMode (not the
        # SelectMode return code). Symmetric with Enter_Debug_Mode's release loop.
        # Returns (ok, name): ok=True once the robot confirms 'ai' mode.
        try:
            for _ in range(retries):
                self.msc.SelectMode(nameOrAlias='ai')
                time.sleep(wait)
                _, check = self.msc.CheckMode()
                name = check.get('name') if isinstance(check, dict) else None
                if name == 'ai':
                    return True, name
            return False, None
        except Exception as e:
            return False, None

class LocoClientWrapper:
    def __init__(self):
        self.client = LocoClient()
        self.client.SetTimeout(0.0001)
        self.client.Init()

    def Enter_Damp_Mode(self):
        self.client.Damp()
    
    def Move(self, vx, vy, vyaw):
        self.client.Move(vx, vy, vyaw, continous_move=False)

if __name__ == '__main__':
    ChannelFactoryInitialize(1) # 0 for real robot, 1 for simulation
    ms = MotionSwitcher()
    status, result = ms.Enter_Debug_Mode()
    print("Enter debug mode:", status, result)
    time.sleep(5)
    status, result = ms.Exit_Debug_Mode()
    print("Exit debug mode:", status, result)
    time.sleep(2)
