""" Transformer 
This class allows for easy traking of 
reference frames. It was developed as
a helper for the Arman project

1/3/2020
Version1.0
"""
import numpy as np

class Transformer:
    def __init__(self, phi, theta, psi, name, protocol=['psi', 'theta', 'phi'], translation=np.array([0.0,0.0,0.0]), parent=None):
        self.name = name
        self.child = None
        self.parent = parent
        self.protocol = protocol
        self.R = np.matrix([[1.0,0.0,0.0],
                           [0.0,1.0,0.0],
                           [0.0,0.0,1.0]])
        self.tf = np.zeros((4,4))
        self.tfI = np.zeros((4,4))
        self.build_transform(phi, theta, psi, translation)
        
    def Rphi(self, phi):
        return np.matrix([[1.0, 0.0, 0.0],
               [0.0, np.cos(phi), np.sin(phi)],
               [0.0, -np.sin(phi), np.cos(phi)]])
        
    def Rtheta(self, theta):
        return np.matrix([[np.cos(theta), 0.0, -np.sin(theta)],
               [0.0, 1.0, 0.0],
               [np.sin(theta), 0.0, np.cos(theta)]])
    
    def Rpsi(self, psi):
        return np.matrix([[np.cos(psi), np.sin(psi), 0.0],
               [-np.sin(psi), np.cos(psi), 0.0],
               [0.0, 0.0, 1.0]])
    
    def build_rotation(self, phi, theta, psi, protocol=None):
        self.R = np.matrix([[1.0,0.0,0.0],
                           [0.0,1.0,0.0],
                           [0.0,0.0,1.0]])
        if protocol is None:
            protocol = self.protocol
        for step in protocol[::-1]:
            if step == 'phi':
                self.R = np.matmul(self.R, self.Rphi(phi))
            elif step == 'theta':
                self.R = np.matmul(self.R, self.Rtheta(theta))
            else:
                self.R = np.matmul(self.R, self.Rpsi(psi))
    
    def add_translation(self, translation, R=None):
        if R is None:
            R = self.R
        n,m = R.shape
        assert len(translation) == n, f'Translation dimension does not equal Rotation dimension: {len(translation)} != {n}'
        new_tf = np.zeros((n+1,m+1))
        for i, row in enumerate(R):
            new_tf[i] = np.concatenate([np.array(row).reshape(m), [translation[i]]])
        new_tf[m,n] = 1.0
        return new_tf

    def build_transform(self, phi, theta, psi, translation):
        self.build_rotation(phi, theta, psi)
        self.tf = self.add_translation(translation, R=self.R)
        self.tfI = self.add_translation(translation, R=self.R.T)
    
    def transform(self, point, inverse=False):
        """ input/output: a 3d cartesian point as numpy array
        """
        if len(point) == 3:
            point = np.concatenate([point, [1.0]])    
        assert point.shape == (4,), 'Illegal point: cannot transform'
        if inverse:
            point = np.array(np.matmul(self.tfI, point.T)).reshape(4) 
            if not self.parent is None:
                return self.parent.transform(point[:3], inverse=True)
        else:
            point = np.array(np.matmul(self.tf, point.T)).reshape(4) 
            if not self.child is None:
                return self.child.transform(point[:3])

        return point[:3]