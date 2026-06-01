import logging
import time
import importlib
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from pprint import pformat
from typing import Any

import torch

_lerobot_module = sys.modules.get("lerobot")
if _lerobot_module is not None and not hasattr(_lerobot_module, "__path__"):
    sys.modules.pop("lerobot")
lr = importlib.import_module("lerobot.scripts.lerobot_record")

from . import robot as _zyarm_robot  # noqa: F401
from . import teleoperator as _zyarm_teleoperator  # noqa: F401
from .profile import train_profiler
from .record_lifecycle import ZyArmRecordLifecycle


@dataclass
class DatasetRecordConfig:
    repo_id: str
    single_task: str
    root: str | Path | None = None
    fps: int | None = None
    episode_time_s: int | float = 60
    reset_time_s: int | float = 60
    episode_warmup_frames: int = 5
    num_episodes: int = 50
    video: bool = True
    push_to_hub: bool = True
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1
    vcodec: str = "libsvtav1"
    streaming_encoding: bool = False
    encoder_queue_maxsize: int = 30
    encoder_threads: int | None = None
    rename_map: dict[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.single_task is None:
            raise ValueError("You need to provide a task as argument in `single_task`.")


@dataclass
class RecordConfig:
    robot: lr.RobotConfig
    dataset: DatasetRecordConfig
    teleop: lr.TeleoperatorConfig | None = None
    policy: lr.PreTrainedConfig | None = None
    display_data: bool = False
    display_ip: str | None = None
    display_port: int | None = None
    display_compressed_images: bool = False
    play_sounds: bool = True
    resume: bool = False
    interpolation_multiplier: int = 1

    def __post_init__(self) -> None:
        policy_path = lr.parser.get_path_arg("policy")
        if policy_path:
            cli_overrides = lr.parser.get_cli_overrides("policy")
            self.policy = lr.PreTrainedConfig.from_pretrained(policy_path, cli_overrides=cli_overrides)
            self.policy.pretrained_path = policy_path

        if self.teleop is None and self.policy is None:
            raise ValueError("Choose a policy, a teleoperator or both to control the robot")

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        return ["policy"]


def _state_age_at_time_ms(
    state_timestamp: float | None,
    now: float,
    *,
    fallback_age_ms: Any = None,
    fallback_elapsed_ms: float = 0.0,
) -> float | None:
    if state_timestamp is not None:
        try:
            return max(0.0, (float(now) - float(state_timestamp)) * 1000.0)
        except (TypeError, ValueError):
            pass
    if fallback_age_ms is None:
        return None
    try:
        return max(0.0, float(fallback_age_ms) + float(fallback_elapsed_ms))
    except (TypeError, ValueError):
        return None


@lr.safe_stop_image_writer
def record_loop(
    robot: lr.Robot,
    events: dict,
    fps: int,
    teleop_action_processor: lr.RobotProcessorPipeline[
        tuple[lr.RobotAction, lr.RobotObservation], lr.RobotAction
    ],
    robot_action_processor: lr.RobotProcessorPipeline[
        tuple[lr.RobotAction, lr.RobotObservation], lr.RobotAction
    ],
    robot_observation_processor: lr.RobotProcessorPipeline[
        lr.RobotObservation, lr.RobotObservation
    ],
    dataset: lr.LeRobotDataset | None = None,
    teleop: lr.Teleoperator | list[lr.Teleoperator] | None = None,
    policy: lr.PreTrainedPolicy | None = None,
    preprocessor: lr.PolicyProcessorPipeline[dict[str, Any], dict[str, Any]] | None = None,
    postprocessor: lr.PolicyProcessorPipeline[lr.PolicyAction, lr.PolicyAction] | None = None,
    control_time_s: int | float | None = None,
    single_task: str | None = None,
    display_data: bool = False,
    interpolator: lr.ActionInterpolator | None = None,
    display_compressed_images: bool = False,
    record_frames: bool = True,
) -> None:
    if dataset is not None and dataset.fps != fps:
        raise ValueError(f"The dataset fps should be equal to requested fps ({dataset.fps} != {fps}).")

    teleop_arm = teleop_keyboard = None
    if isinstance(teleop, list):
        teleop_keyboard = next((t for t in teleop if isinstance(t, lr.KeyboardTeleop)), None)
        teleop_arm = next(
            (
                t
                for t in teleop
                if isinstance(
                    t,
                    (
                        lr.so_leader.SO100Leader
                        | lr.so_leader.SO101Leader
                        | lr.koch_leader.KochLeader
                        | lr.omx_leader.OmxLeader
                    ),
                )
            ),
            None,
        )

        if not (teleop_arm and teleop_keyboard and len(teleop) == 2 and robot.name == "lekiwi_client"):
            raise ValueError(
                "For multi-teleop, the list must contain exactly one KeyboardTeleop and one arm teleoperator. Currently only supported for LeKiwi robot."
            )

    if policy is not None and preprocessor is not None and postprocessor is not None:
        policy.reset()
        preprocessor.reset()
        postprocessor.reset()

    if interpolator is not None:
        interpolator.reset()

    use_interpolation = interpolator is not None and interpolator.enabled and policy is not None
    control_interval = interpolator.get_control_interval(fps) if interpolator else 1 / fps
    action_keys = sorted(robot.action_features) if use_interpolation else []

    no_action_count = 0
    timestamp = 0.0
    start_episode_t = time.perf_counter()
    while timestamp < control_time_s:
        start_loop_t = time.perf_counter()
        train_obs_ms = 0.0
        train_policy_ms = 0.0
        train_send_ms = 0.0
        train_state_age_at_obs_ms = None
        train_state_timestamp = None
        train_state_age_at_send_ms = None

        if events["exit_early"]:
            events["exit_early"] = False
            break

        obs_start_t = time.perf_counter()
        obs = robot.get_observation()
        obs_done_t = time.perf_counter()
        train_obs_ms = (obs_done_t - obs_start_t) * 1000.0
        train_state_age_at_obs_ms = getattr(robot, "last_observation_state_age_ms", None)
        train_state_timestamp = getattr(robot, "last_observation_state_timestamp", None)
        obs_processed = robot_observation_processor(obs)

        if policy is not None or dataset is not None:
            observation_frame = lr.build_dataset_frame(dataset.features, obs_processed, prefix=lr.OBS_STR)

        is_record_frame = True

        if policy is not None and preprocessor is not None and postprocessor is not None:
            if use_interpolation:
                ran_inference = False

                if interpolator.needs_new_action():
                    policy_start_t = time.perf_counter()
                    action_values = lr.predict_action(
                        observation=observation_frame,
                        policy=policy,
                        device=lr.get_safe_torch_device(policy.config.device),
                        preprocessor=preprocessor,
                        postprocessor=postprocessor,
                        use_amp=policy.config.use_amp,
                        task=single_task,
                        robot_type=robot.robot_type,
                    )
                    train_policy_ms += (time.perf_counter() - policy_start_t) * 1000.0
                    act_processed_policy = lr.make_robot_action(action_values, dataset.features)
                    robot_action_to_send = robot_action_processor((act_processed_policy, obs))

                    action_tensor = torch.tensor([robot_action_to_send[k] for k in action_keys])
                    interpolator.add(action_tensor)
                    ran_inference = True

                interp_action = interpolator.get()
                if interp_action is None:
                    continue
                robot_action_to_send = {k: interp_action[i].item() for i, k in enumerate(action_keys)}
                action_values = robot_action_to_send
                is_record_frame = ran_inference
            else:
                policy_start_t = time.perf_counter()
                action_values = lr.predict_action(
                    observation=observation_frame,
                    policy=policy,
                    device=lr.get_safe_torch_device(policy.config.device),
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    use_amp=policy.config.use_amp,
                    task=single_task,
                    robot_type=robot.robot_type,
                )
                train_policy_ms += (time.perf_counter() - policy_start_t) * 1000.0
                act_processed_policy: lr.RobotAction = lr.make_robot_action(action_values, dataset.features)
                robot_action_to_send = robot_action_processor((act_processed_policy, obs))
                action_values = robot_action_to_send

        elif policy is None and isinstance(teleop, lr.Teleoperator):
            act = teleop.get_action()
            if robot.name == "unitree_g1":
                teleop.send_feedback(obs)

            act_processed_teleop = teleop_action_processor((act, obs))
            action_values = act_processed_teleop
            robot_action_to_send = robot_action_processor((act_processed_teleop, obs))

        elif policy is None and isinstance(teleop, list):
            arm_action = teleop_arm.get_action()
            arm_action = {f"arm_{k}": v for k, v in arm_action.items()}
            keyboard_action = teleop_keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_action)
            act = {**arm_action, **base_action} if len(base_action) > 0 else arm_action
            act_processed_teleop = teleop_action_processor((act, obs))
            action_values = act_processed_teleop
            robot_action_to_send = robot_action_processor((act_processed_teleop, obs))
        else:
            no_action_count += 1
            if no_action_count == 1 or no_action_count % 10 == 0:
                logging.warning(
                    "No policy or teleoperator provided, skipping action generation. "
                    "This is likely to happen when resetting the environment without a teleop device. "
                    "The robot won't be at its rest position at the start of the next episode."
                )
            continue

        send_start_t = time.perf_counter()
        train_state_age_at_send_ms = _state_age_at_time_ms(
            train_state_timestamp,
            send_start_t,
            fallback_age_ms=train_state_age_at_obs_ms,
            fallback_elapsed_ms=(send_start_t - obs_done_t) * 1000.0,
        )
        robot.send_action(robot_action_to_send)
        train_send_ms = (time.perf_counter() - send_start_t) * 1000.0

        if dataset is not None and record_frames and is_record_frame:
            action_frame = lr.build_dataset_frame(dataset.features, action_values, prefix=lr.ACTION)
            frame = {**observation_frame, **action_frame, "task": single_task}
            dataset.add_frame(frame)

        if display_data:
            lr.log_rerun_data(
                observation=obs_processed, action=action_values, compress_images=display_compressed_images
            )

        dt_s = time.perf_counter() - start_loop_t
        if policy is not None:
            train_profiler.sample(
                loop_ms=dt_s * 1000.0,
                obs_ms=train_obs_ms,
                policy_ms=train_policy_ms,
                send_ms=train_send_ms,
                state_age_at_obs_ms=train_state_age_at_obs_ms,
                state_age_at_send_ms=train_state_age_at_send_ms,
                control_interval_ms=control_interval * 1000.0,
                state_max_age_ms=getattr(getattr(robot, "config", None), "state_max_age_ms", None),
                arm=getattr(robot, "arm", None),
            )
        sleep_time_s = control_interval - dt_s
        if sleep_time_s < 0:
            logging.warning(
                f"Record loop is running slower ({1 / dt_s:.1f} Hz) than the target FPS from --dataset.fps ({fps} Hz). Dataset frames might be dropped and robot control might be unstable. Common causes are: 1) Camera FPS not keeping up 2) Policy inference taking too long 3) CPU starvation"
            )

        lr.precise_sleep(max(sleep_time_s, 0.0))
        timestamp = time.perf_counter() - start_episode_t


def should_run_zyarm_reset_follow(
    robot: lr.Robot, teleop: lr.Teleoperator | list[lr.Teleoperator] | None
) -> bool:
    return (
        robot.name == "zyarm_follower"
        and isinstance(teleop, lr.Teleoperator)
        and getattr(teleop, "name", None) == "zyarm_leader"
    )


def zyarm_reset_follow_loop(
    *,
    robot: lr.Robot,
    teleop: lr.Teleoperator,
    events: dict,
    fps: int,
    teleop_action_processor: lr.RobotProcessorPipeline[
        tuple[lr.RobotAction, lr.RobotObservation], lr.RobotAction
    ],
    robot_action_processor: lr.RobotProcessorPipeline[
        tuple[lr.RobotAction, lr.RobotObservation], lr.RobotAction
    ],
    reset_time_s: int | float,
    episode_index: int,
) -> None:
    control_interval = 1 / fps
    start_t = time.perf_counter()
    while time.perf_counter() - start_t < reset_time_s:
        loop_t = time.perf_counter()

        if events["stop_recording"] or events["exit_early"]:
            events["exit_early"] = False
            break

        try:
            act = teleop.get_action()
            empty_observation: lr.RobotObservation = {}
            act_processed = teleop_action_processor((act, empty_observation))
            robot_action_to_send = robot_action_processor((act_processed, empty_observation))
            robot.send_action(robot_action_to_send)
        except Exception as exc:
            logging.error(
                "ZYArm record stop: stage=C_RESET_FOLLOW episode_index=%s reason=%s",
                episode_index,
                exc,
            )
            events["stop_recording"] = True
            raise

        sleep_time_s = control_interval - (time.perf_counter() - loop_t)
        if sleep_time_s > 0:
            lr.precise_sleep(sleep_time_s)


@lr.parser.wrap()
def record(cfg: RecordConfig) -> lr.LeRobotDataset:
    lr.init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        lr.init_rerun(session_name="recording", ip=cfg.display_ip, port=cfg.display_port)
    display_compressed_images = (
        True
        if (cfg.display_data and cfg.display_ip is not None and cfg.display_port is not None)
        else cfg.display_compressed_images
    )

    robot = lr.make_robot_from_config(cfg.robot)
    teleop = lr.make_teleoperator_from_config(cfg.teleop) if cfg.teleop is not None else None

    dataset_fps = cfg.dataset.fps
    if dataset_fps is None:
        follower_hz = getattr(cfg.robot, "follower_hz", None)
        if follower_hz is None:
            raise ValueError("dataset.fps is not set and robot config has no follower_hz")
        dataset_fps = int(follower_hz)
        if dataset_fps <= 0:
            raise ValueError(f"follower_hz must be positive, got {follower_hz}")

    teleop_action_processor, robot_action_processor, robot_observation_processor = lr.make_default_processors()

    dataset_features = lr.combine_feature_dicts(
        lr.aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=lr.create_initial_features(action=robot.action_features),
            use_videos=cfg.dataset.video,
        ),
        lr.aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=lr.create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    dataset = None
    listener = None

    try:
        if cfg.resume:
            num_cameras = len(robot.cameras) if hasattr(robot, "cameras") else 0
            dataset = lr.LeRobotDataset.resume(
                cfg.dataset.repo_id,
                root=cfg.dataset.root,
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
                image_writer_processes=cfg.dataset.num_image_writer_processes if num_cameras > 0 else 0,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras
                if num_cameras > 0
                else 0,
            )
            lr.sanity_check_dataset_robot_compatibility(dataset, robot, dataset_fps, dataset_features)
        else:
            lr.sanity_check_dataset_name(cfg.dataset.repo_id, cfg.policy)
            dataset = lr.LeRobotDataset.create(
                cfg.dataset.repo_id,
                dataset_fps,
                root=cfg.dataset.root,
                robot_type=robot.name,
                features=dataset_features,
                use_videos=cfg.dataset.video,
                image_writer_processes=cfg.dataset.num_image_writer_processes,
                image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * len(robot.cameras),
                batch_encoding_size=cfg.dataset.video_encoding_batch_size,
                vcodec=cfg.dataset.vcodec,
                streaming_encoding=cfg.dataset.streaming_encoding,
                encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
                encoder_threads=cfg.dataset.encoder_threads,
            )

        policy = None if cfg.policy is None else lr.make_policy(cfg.policy, ds_meta=dataset.meta)
        preprocessor = None
        postprocessor = None
        interpolator = None
        if cfg.policy is not None:
            preprocessor, postprocessor = lr.make_pre_post_processors(
                policy_cfg=cfg.policy,
                pretrained_path=cfg.policy.pretrained_path,
                dataset_stats=lr.rename_stats(dataset.meta.stats, cfg.dataset.rename_map),
                preprocessor_overrides={
                    "device_processor": {"device": cfg.policy.device},
                    "rename_observations_processor": {"rename_map": cfg.dataset.rename_map},
                },
            )
            if cfg.interpolation_multiplier > 1:
                interpolator = lr.ActionInterpolator(multiplier=cfg.interpolation_multiplier)
                logging.info(f"Action interpolation enabled: {cfg.interpolation_multiplier}x control rate")

        robot.connect()
        if teleop is not None:
            teleop.connect()

        listener, events = lr.init_keyboard_listener()

        if not cfg.dataset.streaming_encoding:
            logging.info(
                "Streaming encoding is disabled. If you have capable hardware, consider enabling it for faster episode saving. For libsvtav1 software AV1 at 50Hz, keep streaming encoding disabled so encoding runs after each episode."
            )

        lifecycle = None
        if should_run_zyarm_reset_follow(robot, teleop):
            lifecycle = ZyArmRecordLifecycle(
                warmup_frames=cfg.dataset.episode_warmup_frames,
                reset_time_s=cfg.dataset.reset_time_s,
                control_hz=float(dataset_fps),
            )

        with lr.VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < cfg.dataset.num_episodes and not events["stop_recording"]:
                if lifecycle is not None:
                    lifecycle.run_pre_roll_warmup(
                        teleop=teleop,
                        robot=robot,
                        episode_index=dataset.num_episodes,
                        finalize=None,
                    )
                else:
                    prepare_episode = getattr(teleop, "prepare_episode", None)
                    if callable(prepare_episode):
                        prepare_episode()
                    if cfg.dataset.episode_warmup_frames > 0:
                        record_loop(
                            robot=robot,
                            events=events,
                            fps=dataset_fps,
                            teleop_action_processor=teleop_action_processor,
                            robot_action_processor=robot_action_processor,
                            robot_observation_processor=robot_observation_processor,
                            teleop=teleop,
                            policy=policy,
                            preprocessor=preprocessor,
                            postprocessor=postprocessor,
                            dataset=dataset,
                            record_frames=False,
                            control_time_s=cfg.dataset.episode_warmup_frames / dataset_fps,
                            single_task=cfg.dataset.single_task,
                            display_data=False,
                            interpolator=interpolator,
                            display_compressed_images=display_compressed_images,
                        )
                lr.log_say(f"Recording episode {dataset.num_episodes}", cfg.play_sounds)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=dataset_fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    policy=policy,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    dataset=dataset,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    display_data=cfg.display_data,
                    interpolator=interpolator,
                    display_compressed_images=display_compressed_images,
                )

                if not events["stop_recording"] and (
                    (recorded_episodes < cfg.dataset.num_episodes - 1) or events["rerecord_episode"]
                ):
                    lr.log_say("Reset the environment", cfg.play_sounds)

                    if should_run_zyarm_reset_follow(robot, teleop):
                        zyarm_reset_follow_loop(
                            robot=robot,
                            teleop=teleop,
                            events=events,
                            fps=dataset_fps,
                            teleop_action_processor=teleop_action_processor,
                            robot_action_processor=robot_action_processor,
                            reset_time_s=cfg.dataset.reset_time_s,
                            episode_index=dataset.num_episodes,
                        )
                    else:
                        reset_start_t = time.perf_counter()
                        while time.perf_counter() - reset_start_t < cfg.dataset.reset_time_s:
                            if events["stop_recording"] or events["exit_early"]:
                                events["exit_early"] = False
                                break
                            lr.precise_sleep(0.1)

                if events["rerecord_episode"]:
                    lr.log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
    finally:
        lr.log_say("Stop recording", cfg.play_sounds, blocking=True)

        if dataset:
            dataset.finalize()

        if robot.is_connected:
            robot.disconnect()
        if teleop and teleop.is_connected:
            teleop.disconnect()

        if not lr.is_headless() and listener:
            listener.stop()

        if cfg.dataset.push_to_hub:
            dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)

        lr.log_say("Exiting", cfg.play_sounds)
    return dataset


def main() -> None:
    lr.register_third_party_plugins()
    record()
