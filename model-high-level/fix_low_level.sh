#!/bin/bash

echo "Adding TYPE comments to generated SCXML files..."
echo ""
# takes as input the folder containing the generated SCXML files

folder=$1

if [ -z "$folder" ]; then
    echo "Usage: $0 <folder>"
    exit 1
fi
echo "Fixing ros_fields__speech_time type annotations..."
for file in "$folder"/*.scxml; do
    if grep -q 'name="ros_fields__speech_time"' "$file"; then
        echo "  Editing: $file"
        sed -i '/name="ros_fields__speech_time"/i\        <!-- TYPE ros_fields__speech_time:float32 -->' "$file"
    fi
done

echo ""
echo "Fixing ros_fields__angle type annotations..."
for file in "$folder"/*.scxml; do
    if grep -q 'name="ros_fields__angle"' "$file"; then
        echo "  Editing: $file"
        sed -i '/name="ros_fields__angle"/i\        <!-- TYPE ros_fields__angle:float64 -->' "$file"
    fi
done

echo ""
echo "Fixing strict random lower-bound guards (0.0 < __RAND__)..."
for file in "$folder"/*.scxml; do
    if grep -q '0.0 &lt; __RAND__' "$file"; then
        echo "  Editing: $file"
        sed -i 's/0.0 \&lt; __RAND__/0.0 \&lt;= __RAND__/g' "$file"
    fi
done

echo ""
echo "Type annotation fixes completed!"
