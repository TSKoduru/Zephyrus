"""Software contracts, not a drone dynamics simulation or hardware validation."""
from pathlib import Path
import unittest
import numpy as np
from course_localization.controller import TimedPath, CourseFollower
from course_localization.landing_finish import LandingFinish

ROUTE = Path(__file__).resolve().parents[1]/'trajectory/before_tunnel.npz'


class BeforeTunnelTest(unittest.TestCase):
    def test_reference_stays_before_tunnel_and_speed_bounded(self):
        path = TimedPath.load(ROUTE,max_speed=.3048)
        np.testing.assert_allclose(path.position[-1],[2.0,1.70,.116])
        speed=np.linalg.norm(np.diff(path.position,axis=0),axis=1)/np.diff(path.time)
        self.assertLessEqual(speed.max(),.25+1e-10)
        # Once approaching the eastern tunnel, entire envelope stays on entry side.
        near=path.position[:,0]>1.2
        self.assertTrue(np.all(path.position[near,1]-.335>1.219))

    def test_feedback_and_touchdown_three_initial_offsets(self):
        for offset in [0.,.03,-.03]:
            path=TimedPath.load(ROUTE,max_speed=.3048)
            c=CourseFollower(path,max_speed=.3048,max_position_error=.10)
            c.reset(0.)
            land=LandingFinish(path.position[-1],.3048)
            pose=np.r_[path.position[0]+[offset,0,0],path.yaw[0]]
            done=False
            # Ideal kinematic integration exercises software sequencing only.
            for now in np.arange(.05,path.duration+30,.05):
                out=c.update(now,.05,pose,pose[3],measurement_time=now,odom_time=now)
                self.assertFalse(out['stop_requested'])
                landed=bool(pose[2]<=path.ground_z+1e-9)
                state,v=land.update(now,pose,out['reference_position'],landed,out['velocity_enu'])
                self.assertNotEqual(state,'landing_fault')
                self.assertLessEqual(np.linalg.norm(v),.3048+1e-10)
                if state=='landed':done=True;break
                pose[:3]+=np.asarray(v)*.05
                pose[2]=max(path.ground_z,pose[2])
                pose[3]+=out['yaw_rate']*.05
            self.assertTrue(done)
            self.assertLess(np.linalg.norm(pose[:2]-path.position[-1,:2]),.10)

    def test_ground_start_does_not_complete_and_timeout_latches_upstream(self):
        endpoint=np.array([2.0,1.70,.116]);f=LandingFinish(endpoint,.3048)
        self.assertEqual(f.update(0,endpoint,endpoint,True,[0,0,0])[0],'following')
        pose=endpoint+[0,0,.1]
        self.assertEqual(f.update(1,pose,pose,False,[.3,0,0])[0],'landing')
        self.assertEqual(f.update(14,pose,pose,False,[0,0,0])[0],'landing_fault')
        self.assertEqual(f.update(15,endpoint,endpoint,True,[0,0,0])[0],'landed')
        self.assertEqual(f.update(16,pose,pose,False,[1,1,1])[0],'landed')


if __name__=='__main__':unittest.main()
