# Repository-wide mapping and navigation requirements

The user requires every mapping, localization, navigation, ground-alignment,
filtering and TF fix to apply globally to all maps, including newly saved maps,
retained versions and maps selected later. This requirement also applies to
future changes. Never hardcode a map name, path, coordinates or a floor height
to repair one map in place of fixing the shared processing path.

- Put geometry and coordinate corrections in shared functions used by save,
  localization and navigation. Keep PCD, map grid, ground model, body-height
  initialization and TF in the same coordinate system.
- Preserve original saved bundles. Publish a new bundle or runtime copy only
  after every required file and geometry check succeeds.
- Do not change live mapping density, motion control or localization acceptance
  limits as an incidental effect of another fix.
- Verify raw body-origin maps, already aligned maps, at least two named maps,
  repeated loading and both localization/navigation launch paths. Check actual
  PCD geometry and body/footprint heights, not only configuration dictionaries.
- Report synthetic/ROS checks separately from actual robot validation.
