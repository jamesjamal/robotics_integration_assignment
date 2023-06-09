#!/usr/bin/env python3

from copy import deepcopy
import pathlib
import pickle
import time
from typing import List, Tuple

import numpy as np

import cv_bridge
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import Image
import rospy
from gem_lanenet.lanenet_w_line_fit import LaneNetWLineFit

from gem_scenario_runner import euler_to_quat, pose_to_xy_yaw, pause_physics, unpause_physics, \
    control_pure_pursuit, control_stanley, to_front_axle, dynamics, \
    set_model_pose, set_light_properties, get_uniform_random_light_level, \
    LaneDetectScene, State, Percept, state_list_to_ndarray, percept_list_to_ndarray

PLOT_NUM = 3
PLOT_SEP = 30.0  # meter
HALF_LANE_WIDTH = 2.0  # meter
LANE_START, LANE_STOP = 2.0, 32.0  # meter
BOT_Z = -0.11  # meter

PSI_LIM = np.pi / 12  # radian. 15 degrees
CTE_LIM = 1.2  # meter

CURVE_PATH_RADIUS = np.inf
CURVE_PATH_CURVATURE = 0


def check_ground_truth(psi: float, cte: float, pose: Pose) -> bool:
    """ Check generated poses correspond to the given ground truth.
        Note that this is constructed so that there shouldn't be
        any floating point error.
    """
    # TODO
    return True


def get_uniform_random_scene(truth: Percept) -> LaneDetectScene:
    """ Get a poses for a given ground truth psi and cte
    The samples are drawn first from a discrete choice among three lanes and
    then from a continuous uniform distribution between [LANE_START, LANE_STOP].

    Parameters
    ----------
    truth : Percept
        Heading angle error in radian and Cross track error range in meter
    """
    psi, cte = truth.yaw_err, truth.offset
    x = np.random.uniform(LANE_START, LANE_STOP)
    road_id = np.random.choice(PLOT_NUM)
    road_y = PLOT_SEP * road_id
    y = road_y - cte
    z = BOT_Z
    yaw = -psi
    rospy.loginfo("Sampled (x, y, yaw) = (%f, %f, %f) " % (x, y, yaw) +
                  "for ground truth (d, ψ) = (%f, %f)" % (cte, psi))
    pose = Pose(position=Point(x, y, z),
                orientation=euler_to_quat(yaw=yaw))
    if not check_ground_truth(psi, cte, pose):
        rospy.logwarn("The pose does not map to the ground truth.")
    return LaneDetectScene(
        light_level=get_uniform_random_light_level(),
        pose=pose)


class ResetPose:
    """
    This class defines the function to reset a Model to a new pose.
    """

    def __init__(self, model_name: str, lanenet: LaneNetWLineFit):
        self.__model_name = model_name
        self.__light_name = "sundir"
        self._bridge = cv_bridge.CvBridge()
        self._lanenet = lanenet

        self._prev_perceived = Percept(0.0, 0.0, CURVE_PATH_CURVATURE)

    def set_scene(self, scene: LaneDetectScene):
        set_light_properties(self.__light_name, scene.light_level)
        self.set_model_pose(scene.pose)

    def set_model_pose(self, pose: Pose) -> None:
        set_model_pose(self.__model_name, "world", pose)

    def perception(self, ros_img_msg: Image) -> Percept:
        curr_perceived = self._raw_perception(ros_img_msg)
        if not any(np.isnan(curr_perceived)):
            self._prev_perceived = curr_perceived
            return curr_perceived
        else:
            assert not any(np.isnan(self._prev_perceived))
            return self._prev_perceived

    def _raw_perception(self, ros_img_msg: Image) -> Percept:
        """ Given camera image, run LaneNet to estimate the heading and distance """

        cv_image = self._bridge.imgmsg_to_cv2(ros_img_msg, "bgr8")

        center_line, annotated_img = self._lanenet.detect(cv_image)
        if center_line is None:
            rospy.logwarn("Cannot infer the lane center line in the current image. Skip.")
            return Percept(np.nan, np.nan, np.nan)

        # calculate error w.r.t the chosen frame of reference of the vehicle (ego view).
        # NOTE coefficients are in the order of y = c[0] + c[1]*x (+ ... + c[n]*x^n)
        # where x-axis is the forward direction of the ego vehicle
        yaw_err = np.arctan(center_line.convert().coef[1])

        # Calculate the offset as the distance from the chosen frame to lane center line
        # NOTE In this simulation set up we always assume the perception is w.r.t rear_axle
        # base_footprint or base_link is the origin (0.0, 0.0)
        y_diff = center_line(0.0) - 0.0
        offset = y_diff * np.cos(yaw_err)
        return Percept(yaw_err=yaw_err, offset=offset, curvature=CURVE_PATH_CURVATURE)


class ImageBuffer:
    def __init__(self):
        self._curr_msg = None
        self._prev_data_hash = hash(None)

    def wait_and_get(self) -> Image:
        while self._curr_msg is None or hash(self._curr_msg.data) == self._prev_data_hash:
            pass
        msg = deepcopy(self._curr_msg)  # Must have local copy before getting hash
        self._prev_data_hash = hash(msg.data)
        return msg

    def cb(self, msg: Image) -> None:
        self._curr_msg = msg


def main() -> None:
    rospy.init_node("sim_traces", anonymous=True)

    model_name = rospy.get_param("~gazebo_model_name")
    controller = rospy.get_param("~controller", "stanley")
    config_path = rospy.get_param("~config_path")  # file path
    weights_path = rospy.get_param("~weights_path")  # file path
    out_dir = rospy.get_param("~out_dir", "")  # file path
    fields = rospy.get_param("~fields")
    truth_list = rospy.get_param("~truth_list", [])
    max_trace_len = rospy.get_param("~max_trace_len")

    if "truth" not in fields or fields["truth"] != ["cte", "psi"]:
        raise ValueError("Unsupported field declaration %s" % fields)

    init_true_percept_list = [Percept(yaw_err=psi, offset=cte, curvature=CURVE_PATH_CURVATURE)
                              for cte, psi in truth_list]

    img_buffer = ImageBuffer()
    _ = rospy.Subscriber('front_single_camera/image_raw', Image, img_buffer.cb)

    lanenet_detect = LaneNetWLineFit(
        config_path=config_path,
        weights_path=weights_path)
    rp = ResetPose(model_name, lanenet_detect)

    rospy.sleep(0.001)  # Wait for the simulated clock to start

    traces = []  # type: List[Tuple[List[State], List[Percept]]]
    time_usage_img, time_usage_lanenet, time_usage_dynamics = 0.0, 0.0, 0.0
    try:
        for i, init_truth in enumerate(init_true_percept_list):
            scene = get_uniform_random_scene(init_truth)  # New random scene for each trace
            rp.set_scene(scene)
            rospy.loginfo("Trace #%d " % i +
                          "starting from (d*, φ*) = (%f, %f)" % (init_truth.offset, init_truth.yaw_err))
            true_pose_list, perceived_list = [], []
            curr_pose = scene.pose
            true_pose_list.append(deepcopy(curr_pose))
            for _ in range(max_trace_len):
                start_gen_img = time.time()
                rp.set_model_pose(curr_pose)
                unpause_physics()
                ros_img_msg = img_buffer.wait_and_get()
                pause_physics()
                time_usage_img += time.time() - start_gen_img
                start_nnet = time.time()
                prcv_state = rp.perception(ros_img_msg)
                perceived_list.append(prcv_state)
                time_usage_lanenet += time.time() - start_nnet
                rospy.logdebug("Percept (d, φ, κ) = (%f, %f, %f)"
                               % (prcv_state.offset, prcv_state.yaw_err, prcv_state.curvature))
                start_dynamics = time.time()

                if controller == "stanley":
                    front_prcv_state = to_front_axle(prcv_state)
                    next_pose = dynamics(curr_pose, control_stanley(front_prcv_state))
                elif controller == "pure_pursuit":
                    next_pose = dynamics(curr_pose, control_pure_pursuit(prcv_state))
                else:
                    raise RuntimeError

                time_usage_dynamics += time.time() - start_dynamics
                curr_pose = next_pose
                true_pose_list.append(deepcopy(curr_pose))

            # Convert from Gazebo pose to ground truth perception value after whole simulation trace
            true_state_list = [pose_to_xy_yaw(pose) for pose in true_pose_list]
            traces.append((true_state_list, perceived_list))
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    finally:
        rospy.loginfo("Generate images: %f seconds" % time_usage_img)
        rospy.loginfo("NNet Perception: %f seconds" % time_usage_lanenet)
        rospy.loginfo("Compute Next State: %f seconds" % time_usage_dynamics)
        out_path = pathlib.Path(out_dir)
        if out_path.is_dir():
            data = [(state_list_to_ndarray(state_trace), percept_list_to_ndarray(prcv_trace))
                    for state_trace, prcv_trace in traces]
            # Save x, y, yaw in case for sanity check
            time_str = time.strftime("%Y-%m-%d-%H-%M-%S")
            out_pickle_name = 'lanenet_%s_sim_traces_%s.xy_yaw.pickle' % (controller, time_str)
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(data, f)

            def xy_yaw_to_truth(state: State) -> Percept:
                x, y, yaw = state.x, state.y, state.yaw
                yaw_err = -yaw
                offset = (-y + PLOT_SEP/2) % PLOT_SEP - PLOT_SEP/2
                return Percept(yaw_err=yaw_err, offset=offset, curvature=CURVE_PATH_CURVATURE)
            # Save ground truth as well as perceived heading and distance
            converted_trace = [([xy_yaw_to_truth(xy_yaw) for xy_yaw in xy_yaw_trace], prcv_trace)
                               for xy_yaw_trace, prcv_trace in traces]
            data = [(percept_list_to_ndarray(truth_trace), percept_list_to_ndarray(prcv_trace))
                    for truth_trace, prcv_trace in converted_trace]
            out_pickle_name = 'lanenet_%s_sim_traces_%s.psi_cte.pickle' % (controller, time_str)
            out_pickle = out_path.joinpath(out_pickle_name)
            with out_pickle.open('wb') as f:
                pickle.dump(data, f)


if __name__ == "__main__":
    main()
