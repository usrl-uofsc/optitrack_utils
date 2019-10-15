"""
MIT License

Copyright(c) 2018 Brennan Cain and Michail Kalaitzakis(Unmanned Systems and Robotics Lab, University of South Carolina, USA)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author: Michail Kalaitzakis

"""

import numpy as np

class Quaternion:
    """
        Class for rotation quaternions
    """

    def __init__(self, x = 0, y = 0, z = 0, w = 1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def setValues(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def norm(self):
        mag = pow(pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2) + pow(self.w, 2), 0.5)

        if mag > 0:
            self.x = self.x / mag
            self.y = self.y / mag
            self.z = self.z / mag
            self.w = self.w / mag
	
def quatInverse(q):
	return Quaternion(-q.x, -q.y, -q.z, q.w)

def quatMultiply(p, q):
    x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y
    y = p.w * q.y - p.x * q.z + p.y * q.w + p.z * q.x
    z = p.w * q.z + p.x * q.y - p.y * q.x + p.z * q.w
    w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z
    
    return Quaternion(x, y, z, w)

def quat2rpy(q):
    r = np.arctan2(2 * (q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))

    sinP = 2 * (q.w*q.y - q.z*q.x)
    if np.abs(sinP) > 1:
        p = np.sign(sinP) * np.pi / 2
    else:
        p = np.arcsin(sinP)
    
    y = np.arctan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    return np.array([r, p, y])

def rpy2quat(r, p, y):
    cy = np.cos(0.5 * y)
    sy = np.sin(0.5 * y)
    cp = np.cos(0.5 * p)
    sp = np.sin(0.5 * p)
    cr = np.cos(0.5 * r)
    sr = np.sin(0.5 * r)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    q = Quaternion(x, y, z, w)

    return q