# verification

to verify UC3 data you need to

```
as2fm_roaml_to_jani main.xml --scxml-out-dir generated
./fix_low_level.sh generated
cd generated
scan . validate
scan . verify --all
```