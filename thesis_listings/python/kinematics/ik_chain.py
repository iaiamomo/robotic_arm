"""Inverse kinematics — Listing 3.2.

Only the chain initialisation is in the thesis. braccio.urdf is not included.
"""
import ikpy.chain

# Importing the URDF file for robot model and defining active joints
my_chain = ikpy.chain.Chain.from_urdf_file(
    "braccio.urdf",
    active_links_mask=[False, True, True, True, True, True, True]
)
