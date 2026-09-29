from __future__ import annotations

from ifcpath.ifc_loader import _space_is_external


class _FakeSpace:
    def __init__(self, *, boundary=None, predefined=None):
        self.InteriorOrExteriorSpace = boundary
        self.PredefinedType = predefined


def test_ifc2x3_external_space_is_detected() -> None:
    assert _space_is_external(_FakeSpace(boundary="EXTERNAL"))
    assert not _space_is_external(_FakeSpace(boundary="INTERNAL"))


def test_ifc4_external_predefined_type_is_fallback() -> None:
    assert _space_is_external(_FakeSpace(boundary=None, predefined="EXTERNAL"))
    assert not _space_is_external(_FakeSpace(boundary=None, predefined="SPACE"))
