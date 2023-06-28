#!/usr/bin/env python3

import casadi as ca
import casadi_kin_dyn.py3casadi_kin_dyn as cas_kin_dyn
import rospy

if __name__ == "__main__":
    rospy.init_node('cas_kin_dyn', anonymous=True)
    urdf = rospy.get_param('robot_description') # get urdf as string
    kindyn = cas_kin_dyn.CasadiKinDyn(urdf) # construct main class

    fk_str = kindyn.fk('base_link')

    fk = ca.Function.deserialize(fk_str)
    print(fk)

    id_str = kindyn.rnea()
    id = ca.Function.deserialize(id_str)
    print(id)

