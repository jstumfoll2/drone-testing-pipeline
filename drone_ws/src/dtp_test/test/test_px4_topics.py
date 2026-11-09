import pytest

from dtp_test.px4_topics import px4_topic


class FakeV1:
    MESSAGE_VERSION = 1


class FakeUnversioned:
    pass


def test_versioned_topic_gets_suffix():
    assert px4_topic("/fmu/out/vehicle_status", FakeV1) == "/fmu/out/vehicle_status_v1"


def test_version_zero_and_unversioned_have_no_suffix():
    assert px4_topic("/fmu/in/vehicle_command", version=0) == "/fmu/in/vehicle_command"
    assert px4_topic("/fmu/in/vehicle_command", FakeUnversioned) == "/fmu/in/vehicle_command"


def test_negative_version_rejected():
    with pytest.raises(ValueError):
        px4_topic("/x", version=-1)
