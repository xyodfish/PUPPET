#!/usr/bin/env python3
"""
Build demo messages for PUPPET proto definitions.

Usage:
  python3 test/demos/python/proto_message_demo.py \
    --proto-python-root proto/python \
    --out-dir /tmp/puppet_proto_demo
"""

from __future__ import annotations

import argparse
import pathlib
import sys

from google.protobuf import json_format


def _add_proto_path(proto_python_root: pathlib.Path) -> None:
    sys.path.insert(0, str(proto_python_root.resolve()))


def _import_proto_modules():
    try:
        from puppet import common_pb2
        from puppet import control_intent_pb2
        from puppet import primitive_frame_pb2
        from puppet import primitive_trajectory_pb2
        from puppet import uniform_pb2
    except Exception as exc:
        raise RuntimeError(
            "failed to import generated *_pb2 modules.\n"
            "Generate them first, e.g.:\n"
            "  mkdir -p proto/python\n"
            "  protoc --proto_path=proto/proto --python_out=proto/python "
            "proto/proto/puppet/common.proto "
            "proto/proto/puppet/uniform.proto "
            "proto/proto/puppet/primitive_frame.proto "
            "proto/proto/puppet/control_intent.proto "
            "proto/proto/puppet/primitive_trajectory.proto"
        ) from exc
    return (
        common_pb2,
        uniform_pb2,
        primitive_frame_pb2,
        control_intent_pb2,
        primitive_trajectory_pb2,
    )


def _fill_timestamp(ts, sec: int, nanosec: int = 0) -> None:
    ts.sec = sec
    ts.nanosec = nanosec


def _fill_duration(duration, sec_float: float) -> None:
    sec = int(sec_float)
    nanosec = int(round((sec_float - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration.sec = float(sec_float)
    duration.nanosec = sec * 1_000_000_000 + nanosec


def build_primitive_frame(uniform_pb2, primitive_frame_pb2):
    frame = primitive_frame_pb2.PrimitiveFrame()
    _fill_timestamp(frame.header.timestamp, sec=1712736000, nanosec=120000000)
    frame.header.frame_id = "world"
    frame.sequence_id = 42

    frame.context.source_id = "vr_operator_1"
    frame.context.source_type = uniform_pb2.SOURCE_TYPE_VR
    frame.context.semantic_context = "teleop"
    frame.context.mode = "direct_pose"
    frame.context.robot_id = "galbot_alpha"
    frame.context.pipeline_id = "upper_body_default"
    frame.context.tags["operator"] = "alice"

    left_pose = frame.poses.add()
    left_pose.meta.name = "left_hand_pose"
    left_pose.meta.entity = "left_hand"
    left_pose.meta.body_group = uniform_pb2.BODY_GROUP_LEFT_ARM
    left_pose.meta.frame_id = "world"
    left_pose.meta.reference_frame_id = "world"
    _fill_timestamp(left_pose.meta.timestamp, sec=1712736000, nanosec=120000000)
    left_pose.meta.confidence = 0.98
    left_pose.meta.valid = True
    left_pose.pose.position.x = 0.42
    left_pose.pose.position.y = 0.18
    left_pose.pose.position.z = 1.05
    left_pose.pose.orientation.x = 0.0
    left_pose.pose.orientation.y = 0.0
    left_pose.pose.orientation.z = 0.0
    left_pose.pose.orientation.w = 1.0
    left_pose.is_relative = False
    left_pose.target_frame_id = "world"

    grip = frame.scalars.add()
    grip.meta.name = "left_grip"
    grip.meta.entity = "left_gripper"
    grip.meta.body_group = uniform_pb2.BODY_GROUP_LEFT_GRIPPER
    _fill_timestamp(grip.meta.timestamp, sec=1712736000, nanosec=120000000)
    grip.meta.confidence = 1.0
    grip.meta.valid = True
    grip.value = 0.35
    grip.min_value = 0.0
    grip.max_value = 1.0

    clutch = frame.booleans.add()
    clutch.meta.name = "deadman"
    clutch.meta.entity = "operator_switch"
    clutch.meta.body_group = uniform_pb2.BODY_GROUP_WHOLE_BODY
    _fill_timestamp(clutch.meta.timestamp, sec=1712736000, nanosec=120000000)
    clutch.meta.confidence = 1.0
    clutch.meta.valid = True
    clutch.value = True

    base_cmd = frame.planar_motions.add()
    base_cmd.meta.name = "base_cmd"
    base_cmd.meta.entity = "mobile_base"
    base_cmd.meta.body_group = uniform_pb2.BODY_GROUP_BASE
    _fill_timestamp(base_cmd.meta.timestamp, sec=1712736000, nanosec=120000000)
    base_cmd.meta.confidence = 1.0
    base_cmd.meta.valid = True
    base_cmd.vx = 0.3
    base_cmd.vy = 0.0
    base_cmd.wz = 0.1
    base_cmd.reference_frame_id = "base_link"

    frame.tags["demo"] = "primitive_frame"
    return frame


def build_control_intent(uniform_pb2, control_intent_pb2):
    intent = control_intent_pb2.ControlIntent()
    _fill_timestamp(intent.header.timestamp, sec=1712736000, nanosec=130000000)
    intent.header.frame_id = "world"
    intent.sequence_id = 5001

    intent.context.source_id = "runtime_orchestrator"
    intent.context.source_type = uniform_pb2.SOURCE_TYPE_EXTERNAL
    intent.context.semantic_context = "teleop_runtime"
    intent.context.mode = "assist_ik"
    intent.context.robot_id = "galbot_alpha"
    intent.context.pipeline_id = "arm_ik_pipeline"

    arm_group = intent.group_intents.add()
    arm_group.body_group = uniform_pb2.BODY_GROUP_LEFT_ARM
    arm_group.owner_source_id = "vr_operator_1"
    arm_group.mode = "ik_pose"
    arm_group.priority = 100
    arm_group.backend_hint = "IkBackend"
    arm_group.enabled = True

    ee = arm_group.ee_pose_intents.add()
    ee.ee_name = "left_ee"
    ee.reference_frame_id = "world"
    ee.pose.position.x = 0.45
    ee.pose.position.y = 0.22
    ee.pose.position.z = 1.02
    ee.pose.orientation.w = 1.0
    ee.twist_feedforward.linear.x = 0.05
    ee.position_weight = 1.0
    ee.orientation_weight = 0.8
    ee.confidence = 0.95

    joint = arm_group.joint_command_intents.add()
    joint.body_group = uniform_pb2.BODY_GROUP_LEFT_ARM
    joint.mode = control_intent_pb2.JointCommandIntent.JOINT_COMMAND_MODE_POSITION
    joint.joint_names.extend(["l_shoulder_pitch", "l_shoulder_roll", "l_elbow_pitch"])
    joint.position.extend([0.2, -0.1, 0.6])
    joint.stiffness.extend([30.0, 30.0, 20.0])
    joint.damping.extend([2.0, 2.0, 1.5])
    joint.weight = 0.5

    cst = arm_group.constraints.add()
    cst.type = control_intent_pb2.ConstraintRequest.CONSTRAINT_TYPE_JOINT_POSITION_LIMIT
    cst.hard = True
    cst.weight = 1.0
    cst.target = "left_arm"
    cst.scalar_params["margin"] = 0.02

    base_group = intent.group_intents.add()
    base_group.body_group = uniform_pb2.BODY_GROUP_BASE
    base_group.owner_source_id = "vr_operator_1"
    base_group.mode = "manual_base"
    base_group.priority = 80
    base_group.backend_hint = "DirectMappingBackend"
    base_group.enabled = True
    base = base_group.base_motion_intents.add()
    base.reference_frame_id = "base_link"
    base.vx = 0.2
    base.vy = 0.0
    base.wz = 0.1
    base.accel_limit = 0.6

    intent.tags["demo"] = "control_intent"
    return intent


def build_trajectory(uniform_pb2, primitive_trajectory_pb2, frame_a):
    traj = primitive_trajectory_pb2.PrimitiveFrameTrajectory()
    _fill_timestamp(traj.trajectory.header.timestamp, sec=1712736000, nanosec=150000000)
    traj.trajectory.header.frame_id = "world"
    traj.trajectory.context.source_id = "offline_track_player"
    traj.trajectory.context.source_type = uniform_pb2.SOURCE_TYPE_EXTERNAL
    traj.trajectory.context.semantic_context = "replay"
    traj.trajectory.context.mode = "trajectory_playback"
    traj.trajectory.context.robot_id = "galbot_alpha"
    traj.trajectory.context.pipeline_id = "upper_body_default"
    traj.trajectory.trajectory_id = "traj_demo_001"
    traj.trajectory.time_domain = primitive_trajectory_pb2.TIME_DOMAIN_MONOTONIC
    _fill_duration(traj.trajectory.total_duration, 1.0)
    traj.trajectory.looping = False
    traj.trajectory.default_interp = primitive_trajectory_pb2.INTERPOLATION_MODE_SLERP

    p0 = traj.samples.add()
    _fill_duration(p0.t_from_start, 0.0)
    p0.frame.CopyFrom(frame_a)
    p0.frame.sequence_id = 100

    p1 = traj.samples.add()
    _fill_duration(p1.t_from_start, 0.5)
    p1.frame.CopyFrom(frame_a)
    p1.frame.sequence_id = 101
    p1.frame.poses[0].pose.position.x = 0.50
    p1.frame.planar_motions[0].vx = 0.4

    p2 = traj.samples.add()
    _fill_duration(p2.t_from_start, 1.0)
    p2.frame.CopyFrom(frame_a)
    p2.frame.sequence_id = 102
    p2.frame.poses[0].pose.position.x = 0.58
    p2.frame.planar_motions[0].vx = 0.0

    traj.tags["demo"] = "primitive_trajectory"
    return traj


def _write_outputs(out_dir: pathlib.Path, frame, intent, traj) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    (out_dir / "primitive_frame.json").write_text(
        json_format.MessageToJson(frame, preserving_proto_field_name=True, indent=2),
        encoding="utf-8",
    )
    (out_dir / "control_intent.json").write_text(
        json_format.MessageToJson(intent, preserving_proto_field_name=True, indent=2),
        encoding="utf-8",
    )
    (out_dir / "primitive_trajectory.json").write_text(
        json_format.MessageToJson(traj, preserving_proto_field_name=True, indent=2),
        encoding="utf-8",
    )

    (out_dir / "primitive_frame.pb").write_bytes(frame.SerializeToString())
    (out_dir / "control_intent.pb").write_bytes(intent.SerializeToString())
    (out_dir / "primitive_trajectory.pb").write_bytes(traj.SerializeToString())


def main() -> int:
    parser = argparse.ArgumentParser(description="PUPPET proto message demo builder")
    parser.add_argument(
        "--proto-python-root",
        default="proto/python",
        help="directory that contains generated python protobuf modules",
    )
    parser.add_argument(
        "--out-dir",
        default="/tmp/puppet_proto_demo",
        help="output directory for json/pb demo files",
    )
    args = parser.parse_args()

    proto_python_root = pathlib.Path(args.proto_python_root)
    _add_proto_path(proto_python_root)
    (
        _common_pb2,
        uniform_pb2,
        primitive_frame_pb2,
        control_intent_pb2,
        primitive_trajectory_pb2,
    ) = _import_proto_modules()

    frame = build_primitive_frame(uniform_pb2, primitive_frame_pb2)
    intent = build_control_intent(uniform_pb2, control_intent_pb2)
    traj = build_trajectory(uniform_pb2, primitive_trajectory_pb2, frame)
    _write_outputs(pathlib.Path(args.out_dir), frame, intent, traj)
    print(f"demo messages written to: {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
