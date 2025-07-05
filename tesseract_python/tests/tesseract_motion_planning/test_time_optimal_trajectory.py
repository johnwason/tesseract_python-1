from tesseract_robotics.tesseract_command_language import CompositeInstruction, StateWaypoint, WaypointPoly, \
    InstructionPoly, MoveInstruction, Instructions, MoveInstructionType_FREESPACE, StateWaypointPoly,\
    MoveInstructionPoly, InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
    MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint, \
    WaypointPoly_wrap_StateWaypoint, InstructionPoly_wrap_MoveInstruction

from tesseract_robotics.tesseract_time_parameterization import TimeOptimalTrajectoryGeneration, \
    InstructionsTrajectory, TimeOptimalTrajectoryGenerationCompositeProfile
from tesseract_robotics.tesseract_common import ProfileDictionary, FilesystemPath, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
import numpy as np
import os

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

def get_environment():
    env = Environment()
    locator = TesseractSupportResourceLocator()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.srdf"))
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names


def create_straight_trajectory():

    num = 10
    max_ = 2.0
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    program = CompositeInstruction()

    for i in range(num):
        p = np.zeros((6,),dtype=np.float64)
        p[0] = i * (max_/num)
        swp = StateWaypoint(joint_names, p)        
        program.append(InstructionPoly_wrap_MoveInstruction(MoveInstruction(
            WaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE)))

    p = np.zeros((6,),dtype=np.float64)
    p[0] = max_
    swp = StateWaypoint(joint_names, p)
    program.push_back(InstructionPoly_wrap_MoveInstruction(MoveInstruction(
            WaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE)))

    return program

def test_time_parameterization():

    time_parameterization = TimeOptimalTrajectoryGeneration("DEFAULT")

    program = create_straight_trajectory()
    traj = InstructionsTrajectory(program)
    max_velocity = np.array([[2.088, 2.082, 3.27, 3.6, 3.3, 3.078]],dtype=np.float64)
    max_velocity = np.hstack((-max_velocity.T, max_velocity.T))
    max_acceleration = np.array([[ 1, 1, 1, 1, 1, 1]],dtype=np.float64)
    max_acceleration = np.hstack((-max_acceleration.T, max_acceleration.T))
    max_jerk = np.array([[ 1, 1, 1, 1, 1, 1]],dtype=np.float64)
    max_jerk = np.hstack((-max_jerk.T, max_jerk.T))
    profile = TimeOptimalTrajectoryGenerationCompositeProfile()
    profile.path_tolerance=1e-3
    profile.min_angle_change=1e-3
    profile.velocity_limits=max_velocity
    profile.acceleration_limits=max_acceleration
    profiles = ProfileDictionary()
    profiles.addProfile("DEFAULT", "DEFAULT", profile)
    env, manip, joint_names=get_environment()
    program.setManipulatorInfo(manip)
    assert time_parameterization.compute(program, env, profiles)
    instr1 = program[-1]
    instr1_1 = InstructionPoly_as_MoveInstructionPoly(instr1)
    result_wp1 = instr1_1.getWaypoint()
    assert WaypointPoly_as_StateWaypointPoly(result_wp1).getTime() > 1.0
    instr1_2 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    assert WaypointPoly_as_StateWaypointPoly(instr1_2.getWaypoint()).getTime() < 5.0
