import re
import traceback
import os
import numpy as np

from tesseract_robotics.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, ManipulatorInfo
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import SimplePlannerLVSPlanProfile

def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return ""    
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()

def get_environment():
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    env = Environment()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    joint_names = env.getManipulatorManager().getFwdKinematicSolver("manipulator").getJointNames()

    return env, manip_info, joint_names

def test_get_environment():
    get_environment()

def test_interpolatestatewaypoint_jointcart_freespace():
    env, manip_info, joint_names = get_environment()

    request = PlannerRequest()
    request.env = env
    request.env_state = env.getCurrentState()
    fwd_kin = env.getManipulatorManager().getFwdKinematicSolver(manip_info.manipulator)
    wp1 = JointWaypoint(joint_names, np.zeros((7,),dtype=np.float64))
    wp2 = CartesianWaypoint(fwd_kin.calcFwdKin(np.ones((7,),dtype=np.float64)))
    instr1 = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE", manip_info)
    instr2 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "TEST_PROFILE", manip_info)

    profile = SimplePlannerLVSPlanProfile(3.14,0.5,1.57,5)
    composite = profile.generate(instr1,instr2,request,ManipulatorInfo())

    for c in composite:
        assert isMoveInstruction(c)
        assert isStateWaypoint(c.as_MoveInstruction().getWaypoint())
        assert c.as_MoveInstruction().getProfile() == instr2.getProfile()

    mi = composite[-1].as_const_MoveInstruction()
    last_position = mi.getWaypoint().as_const_StateWaypoint().position
    final_pose = fwd_kin.calcFwdKin(last_position)
    assert wp2.isApprox(final_pose, 1e-3)
