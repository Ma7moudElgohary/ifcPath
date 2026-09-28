import json

from ifcpath.models import NavigationModel, NavNode
from ifcpath.package import read_inav, write_inav


def test_inav_roundtrip(tmp_path):
    model = NavigationModel(nodes=[NavNode(id="n", position_m=(1, 2, 3))])
    path = write_inav(model, tmp_path / "test.inav", source_name="x.ifc")
    doc = json.loads(path.read_text())
    assert doc["format"] == "IFCPath INAV"
    assert doc["coordinateSystem"]["units"] == "m"
    loaded = read_inav(path)
    assert loaded.schema_version == model.schema_version
    assert loaded.nodes[0].position_m == (1, 2, 3)
