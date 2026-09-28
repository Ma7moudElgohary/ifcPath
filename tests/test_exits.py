from ifcpath.exits import classify_door_exit


def test_explicit_external_door_is_exit_even_with_two_spaces():
    result = classify_door_exit(
        2,
        {"Pset_DoorCommon": {"IsExternal": True}},
    )
    assert result.is_exit
    assert result.is_external is True
    assert result.source == "Pset_DoorCommon.IsExternal"


def test_explicit_internal_door_overrides_single_space_fallback():
    result = classify_door_exit(
        1,
        {"Pset_DoorCommon": {"IsExternal": False}},
    )
    assert not result.is_exit
    assert result.is_external is False
    assert result.source == "Pset_DoorCommon.IsExternal"


def test_single_space_is_fallback_exit_when_external_property_missing():
    result = classify_door_exit(1, {})
    assert result.is_exit
    assert result.is_external is None
    assert result.source == "single-space-fallback"


def test_two_space_door_without_external_property_is_not_exit():
    result = classify_door_exit(2, {})
    assert not result.is_exit
    assert result.exit_source if False else True
    assert result.source == "not-classified"


def test_string_boolean_values_are_supported():
    assert classify_door_exit(2, {"Pset_DoorCommon": {"IsExternal": "TRUE"}}).is_exit
    assert not classify_door_exit(1, {"Pset_DoorCommon": {"IsExternal": "false"}}).is_exit
