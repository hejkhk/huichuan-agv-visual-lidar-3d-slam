# Loc_MAP

Place one matching Cartographer map set here:

```text
map.yaml
map.pgm
map.pbstream
```
The default localization launcher reads the basename `map`. To use another
basename, for example `floor_1`, keep `floor_1.yaml`, the image referenced by
that YAML, and `floor_1.pbstream` in this directory, then run:

```bash
./START_DUAL_2D_3D_LOCALIZATION.sh floor_1
```

The YAML/PGM pair is the immutable Nav2 planning map. The PBSTREAM is the
frozen Cartographer state used for scan matching and loop closure. They must
come from the same saved mapping session.

To open the CAR UI and automatically start localization plus Nav2 with the
last selected map, run from the project root:

```bash
./START_UI_LOCALIZATION_NAVIGATION.sh
```

On first use with multiple complete maps, choose one in UI Map Management and
press **Use Map**. The selection is persisted under
`~/.cache/huichuan_agv/selected_map` and restored on later launches.
