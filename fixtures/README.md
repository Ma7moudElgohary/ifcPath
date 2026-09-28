# Reference IFC fixtures

Real IFC qualification files are downloaded during CI rather than committed to this repository.

## buildingSMART Duplex Apartment

- File: `Duplex_A_20110907.ifc`
- Schema: IFC2x3
- Provenance: buildingSMART community sample
- License: CC BY 4.0
- CI mirror: `https://raw.githubusercontent.com/andyward/XBimDemo/master/Xbim.TestApp/Duplex_A_20110907.ifc`

This model is used to exercise the complete IFCPath pipeline against a real architectural IFC: IFC parsing, walkable geometry generation, levels/spaces/doors/stairs extraction, INAV export, and qualification metrics.

The CI workflow validates the STEP header before processing so a dead/misconfigured download URL cannot silently feed HTML into IfcOpenShell.
