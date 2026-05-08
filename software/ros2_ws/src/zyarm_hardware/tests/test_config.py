from pathlib import Path

import pytest

from zyarm_hardware.config import ConfigError, load_system_config


def test_load_system_config_rejects_unknown_relation_arm(tmp_path: Path):
    config_path = tmp_path / "system.yaml"
    config_path.write_text(
        """
arms:
  leader:
    port: /dev/ttyUSB0
relationships:
  pair:
    mode: teleop_follow
    leader: leader
    follower: missing
    follow_frequency_hz: 50
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="unknown follower"):
        load_system_config(config_path)


def test_load_system_config_rejects_reused_arm_across_relations(tmp_path: Path):
    config_path = tmp_path / "system.yaml"
    config_path.write_text(
        """
arms:
  left_master:
    port: /dev/ttyUSB0
  left_slave:
    port: /dev/ttyUSB1
  right_slave:
    port: /dev/ttyUSB2
relationships:
  left_pair:
    mode: teleop_follow
    leader: left_master
    follower: left_slave
    follow_frequency_hz: 50
  invalid_pair:
    mode: teleop_follow
    leader: left_master
    follower: right_slave
    follow_frequency_hz: 50
""".strip(),
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="reused by relationships"):
        load_system_config(config_path)


def test_load_system_config_accepts_multiple_independent_pairs(tmp_path: Path):
    config_path = tmp_path / "system.yaml"
    config_path.write_text(
        """
arms:
  left_master:
    port: /dev/ttyUSB0
  left_slave:
    port: /dev/ttyUSB1
  right_master:
    port: /dev/ttyUSB2
  right_slave:
    port: /dev/ttyUSB3
relationships:
  left_pair:
    mode: teleop_follow
    leader: left_master
    follower: left_slave
    follow_frequency_hz: 50
  right_pair:
    mode: teleop_follow
    leader: right_master
    follower: right_slave
    follow_frequency_hz: 45
""".strip(),
        encoding="utf-8",
    )

    config = load_system_config(config_path)

    assert sorted(config.arms) == ["left_master", "left_slave", "right_master", "right_slave"]
    assert sorted(config.relationships) == ["left_pair", "right_pair"]
    assert config.relationships["right_pair"].follow_frequency_hz == 45.0


def test_load_system_config_uses_reset_rts_dtr_quiet_s(tmp_path: Path):
    config_path = tmp_path / "system.yaml"
    config_path.write_text(
        """
arms:
  slave:
    port: /dev/ttyUSB0
    reset_rts_dtr: true
    reset_rts_dtr_quiet_s: 3.0
relationships: {}
""".strip(),
        encoding="utf-8",
    )

    config = load_system_config(config_path)

    assert config.arms["slave"].reset_rts_dtr is True
    assert config.arms["slave"].reset_rts_dtr_quiet_s == 3.0


def test_load_system_config_accepts_legacy_startup_reset_quiet_s_alias(tmp_path: Path):
    config_path = tmp_path / "system.yaml"
    config_path.write_text(
        """
arms:
  slave:
    port: /dev/ttyUSB0
    reset_rts_dtr: true
    startup_reset_quiet_s: 2.5
relationships: {}
""".strip(),
        encoding="utf-8",
    )

    config = load_system_config(config_path)

    assert config.arms["slave"].reset_rts_dtr_quiet_s == 2.5
