import roboticstoolbox as rtb
import numpy as np
import spatialmath as sm

def LRMate200iD4S_gen():
    old = False
    if old:
        lrmate_DH = rtb.DHRobot(
            [
                rtb.RevoluteDH(d=0.330, alpha=np.pi/2),
                rtb.RevoluteDH(a=0.260, alpha=np.pi),
                rtb.RevoluteDH(a=0.02, alpha=-np.pi/2),
                rtb.RevoluteDH(d=0.290, alpha=np.pi/2),
                rtb.RevoluteDH(alpha=-np.pi/2),
                rtb.RevoluteDH()
            ], name="LRMate200iD4s")
        lrmate_DH.tool = sm.SE3(0.25, 0, 0.2) * sm.SE3.Ry(np.pi)
        lrmate_DH.base = sm.SE3(0, 0, -0.3)
    else:
        lrmate_DH = rtb.DHRobot(
            [
                rtb.RevoluteDH(alpha=-np.pi/2),
                rtb.RevoluteDH(a=0.260, alpha=np.pi, offset=-np.pi/2),
                rtb.RevoluteDH(a=-0.02, alpha=np.pi/2, offset=-np.pi),
                rtb.RevoluteDH(d=-0.290, alpha=-np.pi/2),
                rtb.RevoluteDH(alpha=-np.pi/2, offset=-np.pi),
                rtb.RevoluteDH(d=-0.07, alpha=-np.pi)
            ], name="LRMate200iD4s")
        #lrmate_DH.tool = sm.SE3(0, 0, 0.115)
        #lrmate_DH.base = sm.SE3(0, 0, 0.33)

    return lrmate_DH


def joints_fanuc2corke(q):
    if q.ndim == 1:
        q = [q]
    q_adj = q
    q_adj[:, 2] = q[:, 2] + q[:, 1]
    q_adj = q_adj/180*np.pi
    return q_adj
