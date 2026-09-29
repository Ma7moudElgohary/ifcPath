from __future__ import annotations

from ifcpath.ifc_loader import _safe_ifc_attr, _space_is_external


class _ShortSpace:
    def __init__(self, *, boundary=RuntimeError("short IFC argument vector"), predefined=None):
        self._boundary = boundary
        self._predefined = predefined

    def __getattr__(self, name: str):
        if name == "InteriorOrExteriorSpace":
            if isinstance(self._boundary, Exception):
                raise self._boundary
            return self._boundary
        if name == "PredefinedType":
            if isinstance(self._predefined, Exception):
                raise self._predefined
            return self._predefined
        raise AttributeError(name)


def test_safe_ifc_attr_treats_short_argument_vector_as_missing() -> None:
    entity = _ShortSpace()
    assert _safe_ifc_attr(entity, "InteriorOrExteriorSpace") is None


def test_space_external_falls_back_when_legacy_semantic_slot_is_unreadable() -> None:
    entity = _ShortSpace(predefined="EXTERNAL")
    assert _space_is_external(entity) is True


def test_space_external_is_false_when_both_semantic_slots_are_unreadable() -> None:
    entity = _ShortSpace(predefined=RuntimeError("another short slot"))
    assert _space_is_external(entity) is False


def test_space_external_prefers_explicit_interior_exterior_semantics() -> None:
    assert _space_is_external(_ShortSpace(boundary="EXTERNAL", predefined="INTERNAL")) is True
    assert _space_is_external(_ShortSpace(boundary="INTERNAL", predefined="EXTERNAL")) is False
