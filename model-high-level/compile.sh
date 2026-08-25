#!/bin/bash
#

as2fm_roaml_to_jani main.xml --scxml-out-dir generated;
./fix_low_level.sh generated/;
cp Properties/properties_scan.xml generated/properties.xml;