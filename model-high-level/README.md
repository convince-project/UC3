# Verification

To generate and verify the UC3 model, run:

```bash
cd /home/user1/UC3/model-high-level
source /home/user1/as2fm-python-env/bin/activate

as2fm_roaml_to_jani main.xml --scxml-out-dir generated
./fix_low_level.sh generated

cd generated
scan . validate
scan . verify --all -d 1000
```

If you do not want to activate the virtual environment, use the absolute command path:

```bash
cd /home/user1/UC3/model-high-level
/home/user1/as2fm-python-env/bin/as2fm_roaml_to_jani main.xml --scxml-out-dir generated
./fix_low_level.sh generated
cd generated
scan . validate
scan . verify --all -d 1000
```

## Common errors

- `bash: as2fm_roaml_to_jani: command not found`
	- Cause: virtual environment not activated and command not available in PATH.
- `grep: generated/*.scxml: No such file or directory`
	- Cause: generation failed, so the `generated` folder was not created.
- `unknown or unexpected start tag 'ros_service'`
	- Cause: `scan` was executed outside `generated`, so it parsed high-level property files.
