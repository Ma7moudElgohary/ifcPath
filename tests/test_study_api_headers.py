from __future__ import annotations

from ifcpath.study_api import _physical_build_headers


def test_physical_build_headers_expose_browser_gate_invariants() -> None:
    payload = {
        "model": {
            "metadata": {"surface_source": "ifc-physical-raycast"},
            "cells": [{"id": "a"}, {"id": "b"}],
        },
        "qualification": {
            "valid": True,
            "stats": {
                "surface_authoritative": True,
                "surface_navigation_ready": True,
                "surface_exit_unreachable_spaces": 0,
                "surface_vertical_transitions": 2,
            },
        },
    }

    headers = _physical_build_headers(payload)

    assert headers["X-IfcPath-Surface-Source"] == "ifc-physical-raycast"
    assert headers["X-IfcPath-Qualification-Valid"] == "true"
    assert headers["X-IfcPath-Surface-Authoritative"] == "true"
    assert headers["X-IfcPath-Surface-Navigation-Ready"] == "true"
    assert headers["X-IfcPath-Surface-Unreachable-Spaces"] == "0"
    assert headers["X-IfcPath-Surface-Vertical-Transitions"] == "2"
    assert headers["X-IfcPath-Surface-Cell-Count"] == "2"


def test_physical_build_headers_fail_closed_when_fields_are_missing() -> None:
    headers = _physical_build_headers({})

    assert headers["X-IfcPath-Surface-Source"] == ""
    assert headers["X-IfcPath-Qualification-Valid"] == "false"
    assert headers["X-IfcPath-Surface-Authoritative"] == "false"
    assert headers["X-IfcPath-Surface-Navigation-Ready"] == "false"
    assert headers["X-IfcPath-Surface-Unreachable-Spaces"] == "-1"
    assert headers["X-IfcPath-Surface-Vertical-Transitions"] == "0"
    assert headers["X-IfcPath-Surface-Cell-Count"] == "0"
