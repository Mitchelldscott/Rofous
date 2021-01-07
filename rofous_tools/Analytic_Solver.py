import numpy as np

class Analytic_Solver():
    def __init__(self):
        pass
    
    def wrap_angles(self, radians):
        """Wraps radian angle to (-pi, pi]"""
        return (radians + np.pi) % (2 * np.pi) - np.pi
    
    def get_axis(self, pose):
        origin = pose[:3]
        x = origin + self.R(np.array([3,0,0]), ref=pose[3:])
        y = origin + self.R(np.array([0,3,0]), ref=pose[3:])
        z = origin + self.R(np.array([0,0,3]), ref=pose[3:])
        return origin, x, y, z
    
    def T(self, displacement, ref):
        """ translates a pose (ref) w.r.t. a displacement
            displacement can be a 3D vector or 6D pose
            ref should be [x,y,z]"""

        assert len(ref) == 3, 'Invalid Reference: format as (x, y, z)'

        if len(displacement) > 3:
            return np.concatenate([displacement[:3] + ref, displacement[3:]])

        return np.array(displacement[:3] + ref)
    
    def Rx(self, effector, phi):
        
        R = np.matrix([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
        
        if len(effector) > 3:
            return np.concatenate([np.array(np.matmul(R, np.array([effector[:3]]).T)).reshape(3), effector[3:] + ref])

        return np.array(np.matmul(R, effector.T))[0]
    
    def Ry(self, effector, theta):
        
        R = np.matrix([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
        
        if len(effector) > 3:
            return np.concatenate([np.array(np.matmul(R, np.array([effector[:3]]).T)).reshape(3), effector[3:] + ref])

        return np.array(np.matmul(R, effector.T))[0]
    
    def Rz(self, effector, psi):
        
        R = np.matrix([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
        
        if len(effector) > 3:
            return np.concatenate([np.array(np.matmul(R, np.array([effector[:3]]).T)).reshape(3), effector[3:] + ref])

        return np.array(np.matmul(R, effector.T))[0]
            
    
    def R(self, effector, ref):
        """ Rotates a 3D vector (effector) by some angles (ref)
            effector can be a 3D vector or a 6D pose
            if 6D only the x,y,z will be affected. phi, theta, psi need 
            to be updated on their own
            ref must be a list of 3 angles to rotate about each axis 
            default ref is poseActuals angular offsets"""

        assert len(ref) == 3, 'Invalid Reference: format as (phi, theta, psi)'

        m = self.Rx(effector, ref[0])
        m = self.Ry(m, ref[1])
        return self.Rz(m, ref[2])
    
    def get_ee_pose(self, pose, link, ref):
        """ returns the pose of an end effector"""
        return np.concatenate([self.R(link, ref) + pose[:3], ref + pose[3:]])
