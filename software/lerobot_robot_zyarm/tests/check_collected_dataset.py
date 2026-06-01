from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import cv2
import pandas as pd

PASS = "PASS"
SUSPECT = "SUSPECT"
FAIL = "FAIL"
REPORT_DIR = Path(__file__).resolve().parent / "dataset_check_reports"


@dataclass
class Issue:
    level: str
    check: str
    reason: str
    actual: Any = None
    expected: Any = None
    path: str | None = None


@dataclass
class VideoProbe:
    path: str
    camera: str
    frame_count: int | None = None
    fps: float | None = None
    duration: float | None = None
    readable: bool = False
    ffprobe: dict[str, Any] = field(default_factory=dict)
    error: str | None = None


@dataclass
class EpisodeSummary:
    episode_index: int
    row_count: int
    frame_index_min: int | None
    frame_index_max: int | None
    timestamp_min: float | None
    timestamp_max: float | None
    status: str
    issues: list[Issue]


@dataclass
class FileRecord:
    data_path: str
    episode_indices: list[int]
    row_count: int
    frame_index_min: int | None
    frame_index_max: int | None
    timestamp_min: float | None
    timestamp_max: float | None
    episodes: list[EpisodeSummary]
    videos: dict[str, VideoProbe]
    status: str
    issues: list[Issue]


@dataclass
class LevelResult:
    name: str
    status: str
    issues: list[Issue]
    details: dict[str, Any] = field(default_factory=dict)


def combine_status(issues: list[Issue]) -> str:
    if any(issue.level == FAIL for issue in issues):
        return FAIL
    if any(issue.level == SUSPECT for issue in issues):
        return SUSPECT
    return PASS


def issue_to_dict(issue: Issue) -> dict[str, Any]:
    return {
        "level": issue.level,
        "check": issue.check,
        "reason": issue.reason,
        "actual": issue.actual,
        "expected": issue.expected,
        "path": issue.path,
    }


def video_to_dict(video: VideoProbe) -> dict[str, Any]:
    return {
        "path": video.path,
        "camera": video.camera,
        "frame_count": video.frame_count,
        "fps": video.fps,
        "duration": video.duration,
        "readable": video.readable,
        "ffprobe": video.ffprobe,
        "error": video.error,
    }


def episode_to_dict(episode: EpisodeSummary) -> dict[str, Any]:
    return {
        "episode_index": episode.episode_index,
        "row_count": episode.row_count,
        "frame_index_min": episode.frame_index_min,
        "frame_index_max": episode.frame_index_max,
        "timestamp_min": episode.timestamp_min,
        "timestamp_max": episode.timestamp_max,
        "status": episode.status,
        "issues": [issue_to_dict(issue) for issue in episode.issues],
    }


def record_to_dict(record: FileRecord) -> dict[str, Any]:
    return {
        "data_path": record.data_path,
        "episode_indices": record.episode_indices,
        "row_count": record.row_count,
        "frame_index_min": record.frame_index_min,
        "frame_index_max": record.frame_index_max,
        "timestamp_min": record.timestamp_min,
        "timestamp_max": record.timestamp_max,
        "episodes": [episode_to_dict(episode) for episode in record.episodes],
        "videos": {key: video_to_dict(video) for key, video in record.videos.items()},
        "status": record.status,
        "issues": [issue_to_dict(issue) for issue in record.issues],
    }


def level_to_dict(level: LevelResult) -> dict[str, Any]:
    return {
        "name": level.name,
        "status": level.status,
        "issues": [issue_to_dict(issue) for issue in level.issues],
        "details": level.details,
    }


def issue_from_exception(level: str, check: str, reason: str, exc: Exception, path: Path | None = None) -> Issue:
    return Issue(level, check, f"{reason}: {type(exc).__name__}: {exc}", path=str(path) if path else None)


def load_info(dataset_root: Path) -> tuple[dict[str, Any], list[Issue]]:
    info_path = dataset_root / "meta" / "info.json"
    issues: list[Issue] = []
    try:
        return json.loads(info_path.read_text(encoding="utf-8")), issues
    except FileNotFoundError:
        issues.append(Issue(FAIL, "structure", "missing meta/info.json", path=str(info_path)))
    except json.JSONDecodeError as exc:
        issues.append(Issue(FAIL, "metadata", f"cannot parse meta/info.json: {exc}", path=str(info_path)))
    return {}, issues


def scan_dataset(dataset_root: Path) -> tuple[dict[str, Any], dict[str, Any], list[Issue]]:
    dataset_root = dataset_root.resolve()
    info, issues = load_info(dataset_root)
    for relative in ("meta", "data", "videos"):
        path = dataset_root / relative
        if not path.exists():
            issues.append(Issue(FAIL, "structure", f"missing {relative}/", path=str(path)))

    data_files = sorted((dataset_root / "data").glob("chunk-*/file-*.parquet")) if (dataset_root / "data").exists() else []
    episode_files = sorted((dataset_root / "meta" / "episodes").glob("**/*.parquet")) if (dataset_root / "meta" / "episodes").exists() else []
    video_files = sorted((dataset_root / "videos").glob("*/chunk-*/*.mp4")) if (dataset_root / "videos").exists() else []
    video_keys = [key for key, feature in info.get("features", {}).items() if feature.get("dtype") == "video"]

    index = {
        "dataset_root": str(dataset_root),
        "fps": info.get("fps"),
        "data_path": info.get("data_path"),
        "video_path": info.get("video_path"),
        "video_keys": video_keys,
        "total_episodes": info.get("total_episodes"),
        "total_frames": info.get("total_frames"),
        "data_files": [str(path) for path in data_files],
        "episode_files": [str(path) for path in episode_files],
        "video_files": [str(path) for path in video_files],
    }
    if not data_files:
        issues.append(Issue(FAIL, "structure", "no data/chunk-*/file-*.parquet files found", path=str(dataset_root / "data")))
    if not episode_files:
        issues.append(Issue(FAIL, "structure", "no meta/episodes/**/*.parquet files found", path=str(dataset_root / "meta" / "episodes")))
    if not video_keys:
        issues.append(Issue(FAIL, "metadata", "no video features found in meta/info.json", path=str(dataset_root / "meta" / "info.json")))
    return info, index, issues


def data_file_indices(path: Path) -> tuple[int | None, int | None]:
    try:
        chunk = int(path.parent.name.removeprefix("chunk-"))
        file_index = int(path.stem.removeprefix("file-"))
        return chunk, file_index
    except ValueError:
        return None, None


def load_parquet(path: Path, check: str) -> tuple[pd.DataFrame | None, list[Issue]]:
    try:
        return pd.read_parquet(path), []
    except Exception as exc:
        return None, [issue_from_exception(FAIL, check, "cannot read parquet", exc, path)]


def resolve_data_path(dataset_root: Path, info: dict[str, Any], chunk_index: Any, file_index: Any) -> Path:
    template = info.get("data_path") or "data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet"
    return dataset_root / template.format(chunk_index=int(chunk_index), file_index=int(file_index))


def resolve_video_path(dataset_root: Path, info: dict[str, Any], video_key: str, chunk_index: Any, file_index: Any) -> Path:
    template = info.get("video_path") or "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
    return dataset_root / template.format(video_key=video_key, chunk_index=int(chunk_index), file_index=int(file_index))


def shape_width(feature: dict[str, Any]) -> int | None:
    shape = feature.get("shape")
    if isinstance(shape, list) and len(shape) == 1 and isinstance(shape[0], int):
        return shape[0]
    return None


def scalar_value(value: Any) -> Any:
    if hasattr(value, "item"):
        return value.item()
    return value


def vector_width(value: Any) -> int | None:
    if hasattr(value, "shape"):
        shape = getattr(value, "shape")
        if len(shape) == 0:
            return 1
        return int(shape[-1])
    if isinstance(value, (list, tuple)):
        return len(value)
    return 1


def read_episode_metadata(dataset_root: Path, episode_files: list[str]) -> tuple[pd.DataFrame | None, list[Issue]]:
    issues: list[Issue] = []
    frames: list[pd.DataFrame] = []
    for episode_file in episode_files:
        df, read_issues = load_parquet(Path(episode_file), "episode_metadata")
        issues.extend(read_issues)
        if df is not None:
            frames.append(df)
    if not frames:
        return None, issues
    return pd.concat(frames, ignore_index=True), issues


def read_tasks(dataset_root: Path) -> tuple[pd.DataFrame | None, list[Issue]]:
    tasks_path = dataset_root / "meta" / "tasks.parquet"
    if not tasks_path.exists():
        return None, [Issue(FAIL, "structure", "missing meta/tasks.parquet", path=str(tasks_path))]
    return load_parquet(tasks_path, "tasks_metadata")


def load_stats(dataset_root: Path) -> tuple[dict[str, Any], list[Issue]]:
    stats_path = dataset_root / "meta" / "stats.json"
    try:
        return json.loads(stats_path.read_text(encoding="utf-8")), []
    except FileNotFoundError:
        return {}, [Issue(FAIL, "structure", "missing meta/stats.json", path=str(stats_path))]
    except json.JSONDecodeError as exc:
        return {}, [Issue(FAIL, "metadata", f"cannot parse meta/stats.json: {exc}", path=str(stats_path))]


def video_path_for(dataset_root: Path, info: dict[str, Any], video_key: str, data_path: Path) -> Path:
    chunk_index, file_index = data_file_indices(data_path)
    template = info.get("video_path") or "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
    if chunk_index is None or file_index is None:
        return dataset_root / "videos" / video_key / data_path.parent.name / f"{data_path.stem}.mp4"
    return dataset_root / template.format(video_key=video_key, chunk_index=chunk_index, file_index=file_index)


def read_data_summary(path: Path, columns: list[str] | None = None) -> tuple[pd.DataFrame | None, dict[str, Any], list[Issue]]:
    issues: list[Issue] = []
    required = ["episode_index", "frame_index", "timestamp"]
    try:
        df = pd.read_parquet(path, columns=columns)
    except Exception as exc:
        if columns is None:
            return None, {}, [Issue(FAIL, "data_read", f"cannot read parquet: {exc}", path=str(path))]
        try:
            df = pd.read_parquet(path)
            issues.append(Issue(SUSPECT, "data_schema", f"parquet cannot be read with info.features columns: {exc}", path=str(path)))
        except Exception as fallback_exc:
            return None, {}, [Issue(FAIL, "data_read", f"cannot read parquet: {fallback_exc}", path=str(path))]

    missing = [column for column in required if column not in df.columns]
    for column in missing:
        issues.append(Issue(FAIL, "data_schema", f"missing required column {column}", path=str(path)))

    summary = {
        "row_count": int(len(df)),
        "episode_indices": sorted(int(value) for value in df["episode_index"].dropna().unique()) if "episode_index" in df else [],
        "frame_index_min": int(df["frame_index"].min()) if "frame_index" in df and not df.empty else None,
        "frame_index_max": int(df["frame_index"].max()) if "frame_index" in df and not df.empty else None,
        "timestamp_min": float(df["timestamp"].min()) if "timestamp" in df and not df.empty else None,
        "timestamp_max": float(df["timestamp"].max()) if "timestamp" in df and not df.empty else None,
    }
    return df, summary, issues


def ffprobe_video(path: Path) -> VideoProbe:
    camera = path.parent.parent.name
    probe = VideoProbe(path=str(path), camera=camera)
    if not path.exists():
        probe.error = "missing video file"
        return probe
    if shutil.which("ffprobe") is None:
        probe.error = "ffprobe not found"
        return probe

    command = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-count_frames",
        "-show_entries",
        "stream=nb_read_frames,nb_frames,r_frame_rate,avg_frame_rate,duration",
        "-of",
        "json",
        str(path),
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=False, timeout=60)
    except Exception as exc:
        probe.error = f"ffprobe failed: {exc}"
        return probe
    if result.returncode != 0:
        probe.error = result.stderr.strip() or "ffprobe returned an error"
        return probe

    try:
        stream = json.loads(result.stdout).get("streams", [{}])[0]
    except (json.JSONDecodeError, IndexError) as exc:
        probe.error = f"cannot parse ffprobe output: {exc}"
        return probe

    probe.ffprobe = stream
    probe.frame_count = parse_int(stream.get("nb_read_frames")) or parse_int(stream.get("nb_frames"))
    probe.fps = parse_rate(stream.get("avg_frame_rate")) or parse_rate(stream.get("r_frame_rate"))
    probe.duration = parse_float(stream.get("duration"))
    probe.readable = probe.frame_count is not None and probe.frame_count >= 0
    if not probe.readable:
        probe.error = "ffprobe did not return a valid frame count"
    return probe


def parse_int(value: Any) -> int | None:
    try:
        if value in (None, "N/A"):
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def parse_float(value: Any) -> float | None:
    try:
        if value in (None, "N/A"):
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_rate(value: Any) -> float | None:
    if not value or value == "N/A":
        return None
    if isinstance(value, str) and "/" in value:
        numerator, denominator = value.split("/", 1)
        denominator_float = float(denominator)
        return float(numerator) / denominator_float if denominator_float else None
    return parse_float(value)


def verify_sample_frames(video: VideoProbe) -> list[Issue]:
    issues: list[Issue] = []
    if video.frame_count is None or video.frame_count <= 0:
        return issues
    capture = cv2.VideoCapture(video.path)
    if not capture.isOpened():
        return [Issue(FAIL, "video_read", "OpenCV cannot open video", path=video.path)]
    for frame in sorted({0, video.frame_count // 2, video.frame_count - 1}):
        capture.set(cv2.CAP_PROP_POS_FRAMES, frame)
        ok, image = capture.read()
        if not ok or image is None:
            issues.append(Issue(FAIL, "video_read", f"cannot read sample frame {frame}", actual=frame, expected=f"0..{video.frame_count - 1}", path=video.path))
    capture.release()
    return issues


def check_frame_index(df: pd.DataFrame, path: Path, episode_index: int) -> list[Issue]:
    if "frame_index" not in df or df.empty:
        return []
    frame_index = df["frame_index"].tolist()
    expected = list(range(len(frame_index)))
    issues: list[Issue] = []
    if frame_index != expected:
        if frame_index[0] != 0:
            issues.append(Issue(SUSPECT, "frame_index", f"episode {episode_index} frame_index does not start at 0", actual=frame_index[0], expected=0, path=str(path)))
        if len(set(frame_index)) != len(frame_index):
            issues.append(Issue(SUSPECT, "frame_index", f"episode {episode_index} frame_index contains duplicates", path=str(path)))
        if any(current <= previous for previous, current in zip(frame_index, frame_index[1:])):
            issues.append(Issue(SUSPECT, "frame_index", f"episode {episode_index} frame_index is not strictly increasing", path=str(path)))
        if frame_index[-1] != len(frame_index) - 1:
            issues.append(Issue(SUSPECT, "frame_index", f"episode {episode_index} frame_index max does not equal episode row_count - 1", actual=frame_index[-1], expected=len(frame_index) - 1, path=str(path)))
        if not issues:
            issues.append(Issue(SUSPECT, "frame_index", f"episode {episode_index} frame_index is not continuous from 0 to episode row_count - 1", path=str(path)))
    return issues


def check_timestamp(df: pd.DataFrame, declared_fps: float | None, path: Path, episode_index: int) -> list[Issue]:
    if "timestamp" not in df or len(df) < 2:
        return []
    values = [float(value) for value in df["timestamp"].tolist()]
    deltas = [current - previous for previous, current in zip(values, values[1:])]
    issues: list[Issue] = []
    if any(delta < 0 for delta in deltas):
        issues.append(Issue(SUSPECT, "timestamp", f"episode {episode_index} timestamp moves backward", path=str(path)))
    if any(delta == 0 for delta in deltas):
        issues.append(Issue(SUSPECT, "timestamp", f"episode {episode_index} timestamp contains duplicate values", path=str(path)))
    positive = [delta for delta in deltas if delta > 0]
    if positive:
        median_delta = sorted(positive)[len(positive) // 2]
        if any(delta > median_delta * 3 for delta in positive):
            issues.append(Issue(SUSPECT, "timestamp", f"episode {episode_index} timestamp has an obvious jump", actual=max(positive), expected=f"<= {median_delta * 3:.6f}", path=str(path)))
        estimated_fps = 1.0 / median_delta if median_delta else None
        if declared_fps and estimated_fps and abs(estimated_fps - float(declared_fps)) / float(declared_fps) > 0.1:
            issues.append(Issue(SUSPECT, "timestamp", f"episode {episode_index} estimated fps differs from declared fps", actual=round(estimated_fps, 3), expected=declared_fps, path=str(path)))
    return issues


def check_episodes(df: pd.DataFrame, declared_fps: float | None, path: Path) -> list[EpisodeSummary]:
    if "episode_index" not in df or df.empty:
        return []
    episodes: list[EpisodeSummary] = []
    for episode_index, episode_df in df.groupby("episode_index", sort=True):
        issues = check_frame_index(episode_df, path, int(episode_index))
        issues.extend(check_timestamp(episode_df, declared_fps, path, int(episode_index)))
        episodes.append(
            EpisodeSummary(
                episode_index=int(episode_index),
                row_count=int(len(episode_df)),
                frame_index_min=int(episode_df["frame_index"].min()) if "frame_index" in episode_df and not episode_df.empty else None,
                frame_index_max=int(episode_df["frame_index"].max()) if "frame_index" in episode_df and not episode_df.empty else None,
                timestamp_min=float(episode_df["timestamp"].min()) if "timestamp" in episode_df and not episode_df.empty else None,
                timestamp_max=float(episode_df["timestamp"].max()) if "timestamp" in episode_df and not episode_df.empty else None,
                status=combine_status(issues),
                issues=issues,
            )
        )
    return episodes


def check_data_schema(df: pd.DataFrame, info: dict[str, Any], path: Path) -> list[Issue]:
    features = info.get("features", {})
    issues: list[Issue] = []
    required = ["action", "observation.state", "timestamp", "frame_index", "episode_index", "index", "task_index"]
    for column in required:
        if column not in df.columns:
            issues.append(Issue(FAIL, "data_schema", f"missing training column {column}", path=str(path)))
    for feature_name, feature in features.items():
        if feature.get("dtype") == "video" or feature_name not in df.columns or df.empty:
            continue
        expected_width = shape_width(feature)
        actual_width = vector_width(df.iloc[0][feature_name])
        if expected_width is not None and actual_width != expected_width:
            issues.append(Issue(FAIL, "data_schema", f"column {feature_name} shape differs from meta/info.json", actual=actual_width, expected=expected_width, path=str(path)))
    if "index" in df.columns and len(df) > 1:
        values = [int(value) for value in df["index"].tolist()]
        if any(current <= previous for previous, current in zip(values, values[1:])):
            issues.append(Issue(FAIL, "data_schema", "global index is not strictly increasing within parquet file", path=str(path)))
    return issues


def check_level1_file_integrity(dataset_root: Path, index: dict[str, Any], global_issues: list[Issue]) -> LevelResult:
    issues = list(global_issues)
    required_files = ["meta/info.json", "meta/stats.json", "meta/tasks.parquet"]
    for relative in required_files:
        path = dataset_root / relative
        if not path.exists():
            issues.append(Issue(FAIL, "structure", f"missing {relative}", path=str(path)))
    return LevelResult(
        name="LEVEL 1 File integrity",
        status=combine_status(issues),
        issues=issues,
        details={
            "data_files": len(index["data_files"]),
            "video_files": len(index["video_files"]),
            "episode_files": len(index["episode_files"]),
        },
    )


def check_level2_index_consistency(dataset_root: Path, info: dict[str, Any], index: dict[str, Any], records: list[FileRecord]) -> LevelResult:
    issues: list[Issue] = []
    video_keys = index["video_keys"]
    episodes_df, episode_issues = read_episode_metadata(dataset_root, index["episode_files"])
    tasks_df, task_issues = read_tasks(dataset_root)
    stats, stats_issues = load_stats(dataset_root)
    issues.extend(episode_issues)
    issues.extend(task_issues)
    issues.extend(stats_issues)

    if episodes_df is not None:
        required_columns = ["episode_index", "dataset_from_index", "dataset_to_index", "data/chunk_index", "data/file_index"]
        for video_key in video_keys:
            required_columns.extend([
                f"videos/{video_key}/chunk_index",
                f"videos/{video_key}/file_index",
                f"videos/{video_key}/from_timestamp",
                f"videos/{video_key}/to_timestamp",
            ])
        for column in required_columns:
            if column not in episodes_df.columns:
                issues.append(Issue(FAIL, "episode_metadata", f"missing meta/episodes column {column}", path=str(dataset_root / "meta" / "episodes")))

        records_by_path = {Path(record.data_path).resolve(): record for record in records}
        for _, row in episodes_df.iterrows():
            episode_index = int(row["episode_index"]) if "episode_index" in row else None
            if {"data/chunk_index", "data/file_index"}.issubset(episodes_df.columns):
                data_path = resolve_data_path(dataset_root, info, row["data/chunk_index"], row["data/file_index"]).resolve()
                record = records_by_path.get(data_path)
                if record is None:
                    issues.append(Issue(FAIL, "episode_metadata", f"episode {episode_index} points to missing data parquet", path=str(data_path)))
                elif episode_index not in record.episode_indices:
                    issues.append(Issue(FAIL, "episode_metadata", f"episode {episode_index} not present in pointed data parquet", actual=record.episode_indices, expected=episode_index, path=str(data_path)))
                if record is not None and {"dataset_from_index", "dataset_to_index"}.issubset(episodes_df.columns):
                    expected_rows = int(row["dataset_to_index"]) - int(row["dataset_from_index"])
                    actual_rows = next((episode.row_count for episode in record.episodes if episode.episode_index == episode_index), None)
                    if actual_rows != expected_rows:
                        issues.append(Issue(FAIL, "episode_metadata", f"episode {episode_index} row count differs from dataset index span", actual=actual_rows, expected=expected_rows, path=str(data_path)))
            for video_key in video_keys:
                chunk_col = f"videos/{video_key}/chunk_index"
                file_col = f"videos/{video_key}/file_index"
                if chunk_col in episodes_df.columns and file_col in episodes_df.columns:
                    video_path = resolve_video_path(dataset_root, info, video_key, row[chunk_col], row[file_col])
                    if not video_path.exists():
                        issues.append(Issue(FAIL, "episode_metadata", f"episode {episode_index} points to missing video", path=str(video_path)))

    if tasks_df is not None and episodes_df is not None:
        task_count = len(tasks_df)
        for record in records:
            try:
                df = pd.read_parquet(record.data_path, columns=["task_index"])
            except Exception as exc:
                issues.append(issue_from_exception(FAIL, "task_index", "cannot read task_index column", exc, Path(record.data_path)))
                continue
            if "task_index" not in df:
                issues.append(Issue(FAIL, "task_index", "missing task_index column", path=record.data_path))
                continue
            invalid = sorted({int(value) for value in df["task_index"].dropna().unique() if int(value) < 0 or int(value) >= task_count})
            if invalid:
                issues.append(Issue(FAIL, "task_index", "task_index values are missing from meta/tasks.parquet", actual=invalid, expected=f"0..{task_count - 1}", path=record.data_path))

    required_stats = [name for name, feature in info.get("features", {}).items() if feature.get("dtype") != "video"]
    for name in required_stats:
        if name not in stats:
            issues.append(Issue(SUSPECT, "stats", f"stats.json missing feature {name}", path=str(dataset_root / "meta" / "stats.json")))
    return LevelResult("LEVEL 2 LeRobot index/schema consistency", combine_status(issues), issues, {"episodes_metadata_rows": 0 if episodes_df is None else len(episodes_df)})


def sample_indices(length: int) -> list[int]:
    if length <= 0:
        return []
    return sorted({0, length // 2, length - 1})


def check_level3_training_read(dataset_root: Path, episode_indices: list[int]) -> LevelResult:
    issues: list[Issue] = []
    details: dict[str, Any] = {"sampled_indices": [], "sampled_episodes": []}
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except Exception as exc:
        return LevelResult("LEVEL 3 Training read simulation", FAIL, [issue_from_exception(FAIL, "training_read", "cannot import LeRobotDataset", exc)], details)

    try:
        dataset = LeRobotDataset(repo_id=dataset_root.name, root=dataset_root)
    except Exception as exc:
        return LevelResult("LEVEL 3 Training read simulation", FAIL, [issue_from_exception(FAIL, "training_read", "cannot construct LeRobotDataset", exc, dataset_root)], details)

    length = len(dataset)
    for idx in sample_indices(length):
        try:
            item = dataset[idx]
            details["sampled_indices"].append(int(idx))
            for required in ("action", "observation.state", "timestamp", "frame_index", "episode_index", "index", "task_index"):
                if required not in item:
                    issues.append(Issue(FAIL, "training_read", f"dataset item missing {required}", actual=list(item.keys()), expected=required, path=str(dataset_root)))
        except Exception as exc:
            issues.append(issue_from_exception(FAIL, "training_read", f"LeRobotDataset cannot read global index {idx}", exc, dataset_root))

    episodes_df, episode_issues = read_episode_metadata(dataset_root, [str(path) for path in (dataset_root / "meta" / "episodes").glob("**/*.parquet")])
    issues.extend(episode_issues)
    if episodes_df is not None and {"episode_index", "dataset_from_index", "dataset_to_index"}.issubset(episodes_df.columns):
        for _, row in episodes_df.iterrows():
            start = int(row["dataset_from_index"])
            end = int(row["dataset_to_index"])
            for idx in sample_indices(end - start):
                absolute_index = start + idx
                try:
                    dataset[absolute_index]
                    details["sampled_episodes"].append({"episode_index": int(row["episode_index"]), "index": absolute_index})
                except Exception as exc:
                    issues.append(issue_from_exception(FAIL, "training_read", f"LeRobotDataset cannot read episode {int(row['episode_index'])} index {absolute_index}", exc, dataset_root))
    return LevelResult("LEVEL 3 Training read simulation", combine_status(issues), issues, details)


def check_file(dataset_root: Path, info: dict[str, Any], video_keys: list[str], data_path: Path) -> FileRecord:
    data_columns = [key for key, feature in info.get("features", {}).items() if feature.get("dtype") != "video"] or None
    df, summary, issues = read_data_summary(data_path, data_columns)
    issues.extend(check_data_schema(df, info, data_path) if df is not None else [])
    episodes: list[EpisodeSummary] = []
    videos: dict[str, VideoProbe] = {}
    if df is not None:
        episodes = check_episodes(df, parse_float(info.get("fps")), data_path)
        for episode in episodes:
            issues.extend(episode.issues)

    row_count = int(summary.get("row_count", 0))
    for video_key in video_keys:
        path = video_path_for(dataset_root, info, video_key, data_path)
        video = ffprobe_video(path)
        videos[video_key] = video
        if video.error:
            issues.append(Issue(FAIL, "video_probe", video.error, path=str(path)))
        if video.frame_count is not None and video.frame_count != row_count:
            issues.append(Issue(FAIL, "frame_alignment", "data row count differs from video frame count", actual={"data_rows": row_count, "video_frames": video.frame_count, "difference": video.frame_count - row_count}, expected="equal", path=str(path)))
        issues.extend(verify_sample_frames(video))

    counts = {key: video.frame_count for key, video in videos.items() if video.frame_count is not None}
    if len(set(counts.values())) > 1:
        issues.append(Issue(FAIL, "multi_camera_alignment", "camera video frame counts differ", actual=counts, expected="all equal", path=str(data_path)))

    return FileRecord(
        data_path=str(data_path),
        episode_indices=summary.get("episode_indices", []),
        row_count=row_count,
        frame_index_min=summary.get("frame_index_min"),
        frame_index_max=summary.get("frame_index_max"),
        timestamp_min=summary.get("timestamp_min"),
        timestamp_max=summary.get("timestamp_max"),
        episodes=episodes,
        videos=videos,
        status=combine_status(issues),
        issues=issues,
    )


def build_report(dataset_root: Path) -> dict[str, Any]:
    dataset_root = dataset_root.resolve()
    info, index, global_issues = scan_dataset(dataset_root)
    records = [check_file(dataset_root, info, index["video_keys"], Path(path)) for path in index["data_files"]]
    actual_total_frames = sum(record.row_count for record in records)
    actual_episode_indices = sorted({episode for record in records for episode in record.episode_indices})

    if info.get("total_frames") is not None and info.get("total_frames") != actual_total_frames:
        global_issues.append(Issue(SUSPECT, "metadata", "info.total_frames differs from scanned data rows", actual=actual_total_frames, expected=info.get("total_frames"), path=str(dataset_root / "meta" / "info.json")))
    if info.get("total_episodes") is not None and info.get("total_episodes") != len(actual_episode_indices):
        global_issues.append(Issue(SUSPECT, "metadata", "info.total_episodes differs from scanned episode_index values", actual=len(actual_episode_indices), expected=info.get("total_episodes"), path=str(dataset_root / "meta" / "info.json")))

    level1 = check_level1_file_integrity(dataset_root, index, global_issues)
    level2 = check_level2_index_consistency(dataset_root, info, index, records)
    record_issues = [issue for record in records for issue in record.issues]
    level2.issues.extend(record_issues)
    level2.status = combine_status(level2.issues)
    level3 = check_level3_training_read(dataset_root, actual_episode_indices)
    levels = [level1, level2, level3]

    counts = {PASS: 0, SUSPECT: 0, FAIL: 0}
    for record in records:
        counts[record.status] += 1
    final_status = combine_status([Issue(level.status, level.name, "") for level in levels])

    return {
        "dataset_root": str(dataset_root),
        "status": final_status,
        "levels": {level.name: level_to_dict(level) for level in levels},
        "summary": {
            "counts": counts,
            "data_files": len(records),
            "video_files": len(index["video_files"]),
            "episode_files": len(index["episode_files"]),
            "actual_total_frames": actual_total_frames,
            "actual_total_episodes": len(actual_episode_indices),
            "declared_total_frames": info.get("total_frames"),
            "declared_total_episodes": info.get("total_episodes"),
            "fps": info.get("fps"),
            "video_keys": index["video_keys"],
        },
        "global_issues": [issue_to_dict(issue) for issue in global_issues],
        "records": [record_to_dict(record) for record in records],
    }


def write_reports(report: dict[str, Any], output_dir: Path = REPORT_DIR) -> dict[str, str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset_name = Path(report["dataset_root"]).name
    json_path = output_dir / f"{dataset_name}_report.json"
    csv_path = output_dir / f"{dataset_name}_report.csv"
    txt_path = output_dir / f"{dataset_name}_summary.txt"

    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["status", "data_path", "episode_indices", "row_count", "video_frames", "episode_statuses", "issues"])
        writer.writeheader()
        for record in report["records"]:
            writer.writerow({
                "status": record["status"],
                "data_path": record["data_path"],
                "episode_indices": ",".join(map(str, record["episode_indices"])),
                "row_count": record["row_count"],
                "video_frames": json.dumps({key: video["frame_count"] for key, video in record["videos"].items()}, ensure_ascii=False),
                "episode_statuses": json.dumps({episode["episode_index"]: episode["status"] for episode in record.get("episodes", [])}, ensure_ascii=False),
                "issues": " | ".join(f"{issue['level']}:{issue['check']}:{issue['reason']}" for issue in record["issues"]),
            })
    txt_path.write_text(format_summary(report), encoding="utf-8")
    return {"json": str(json_path), "csv": str(csv_path), "txt": str(txt_path)}


def format_summary(report: dict[str, Any]) -> str:
    summary = report["summary"]
    lines = [
        f"Dataset: {report['dataset_root']}",
        f"Final status: {report['status']}",
        f"LEVEL 1 File integrity: {report['levels']['LEVEL 1 File integrity']['status']}",
        f"LEVEL 2 LeRobot index/schema consistency: {report['levels']['LEVEL 2 LeRobot index/schema consistency']['status']}",
        f"LEVEL 3 Training read simulation: {report['levels']['LEVEL 3 Training read simulation']['status']}",
        f"Records: PASS={summary['counts'][PASS]} SUSPECT={summary['counts'][SUSPECT]} FAIL={summary['counts'][FAIL]}",
        f"Frames: actual={summary['actual_total_frames']} declared={summary['declared_total_frames']}",
        f"Episodes: actual={summary['actual_total_episodes']} declared={summary['declared_total_episodes']}",
        "",
    ]
    for level in report.get("levels", {}).values():
        if level["issues"]:
            lines.append(f"{level['name']} issues:")
            for issue in level["issues"]:
                lines.append(f"- {issue['level']} {issue['check']}: {issue['reason']} actual={issue['actual']} expected={issue['expected']} path={issue['path']}")
            lines.append("")
    if report["global_issues"]:
        lines.append("Global issues:")
        for issue in report["global_issues"]:
            lines.append(f"- {issue['level']} {issue['check']}: {issue['reason']} actual={issue['actual']} expected={issue['expected']} path={issue['path']}")
        lines.append("")
    lines.append("File records:")
    for record in report["records"]:
        lines.append(f"- {record['status']} {record['data_path']} rows={record['row_count']} episodes={record['episode_indices']}")
        problem_episodes = [episode for episode in record.get("episodes", []) if episode["status"] != PASS]
        for episode in problem_episodes:
            lines.append(f"  - {episode['status']} episode {episode['episode_index']} rows={episode['row_count']} frame_index={episode['frame_index_min']}..{episode['frame_index_max']} timestamp={episode['timestamp_min']}..{episode['timestamp_max']}")
            for issue in episode["issues"]:
                lines.append(f"    - {issue['level']} {issue['check']}: {issue['reason']} actual={issue['actual']} expected={issue['expected']} path={issue['path']}")
        for issue in record["issues"]:
            if issue["check"] not in {"frame_index", "timestamp"}:
                lines.append(f"  - {issue['level']} {issue['check']}: {issue['reason']} actual={issue['actual']} expected={issue['expected']} path={issue['path']}")
    return "\n".join(lines) + "\n"


def print_summary(report: dict[str, Any], paths: dict[str, str]) -> None:
    summary = report["summary"]
    print(f"Dataset: {report['dataset_root']}")
    print(f"PASS={summary['counts'][PASS]} SUSPECT={summary['counts'][SUSPECT]} FAIL={summary['counts'][FAIL]}")
    print(f"Final status: {report['status']}")
    for level in report.get("levels", {}).values():
        print(f"{level['name']}: {level['status']}")
    print(f"Reports: TXT={paths['txt']} JSON={paths['json']} CSV={paths['csv']}")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only LeRobot collected dataset checker. Local: python tests/check_collected_dataset.py C:\\ZY_ZYArm\\zyarmv1\\data\\demo1. Server: python check_collected_dataset.py /path/to/copied/demo1.",
    )
    parser.add_argument("dataset_root", type=Path, help="LeRobot dataset root containing meta/, data/, and videos/.")
    parser.add_argument("--output-dir", type=Path, default=REPORT_DIR, help="Report directory. Default: tests/dataset_check_reports/.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = make_parser().parse_args(argv)
    report = build_report(args.dataset_root)
    paths = write_reports(report, args.output_dir)
    print_summary(report, paths)
    return 1 if report["status"] == FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())
