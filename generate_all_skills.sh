#!/bin/bash

# Directory containing SCXML files
SKILLS_DIR="/home/user1/UC3/model-high-level/Skills"
MODEL_FILE="/home/user1/UC3/parser-and-code-generator/specifications/full-model.xml"
INTERFACE_FILE="/home/user1/UC3/parser-and-code-generator/specifications/interfaces.xml"
TEMPLATE_PATH="/home/user1/model2code/template_skill"
OUTPUT_BASE="/home/user1/UC3/src/skills"

# Ensure the output base directory exists
mkdir -p "$OUTPUT_BASE"

# Process each SCXML file
for scxml_file in "$SKILLS_DIR"/*.scxml; do
    # Skip if not a file
    [ -f "$scxml_file" ] || continue
    
    # Get the base name without extension
    filename=$(basename "$scxml_file")
    skill_name="${filename%.scxml}"
    
    # Remove "Skill" from the name if present
    skill_name="${skill_name%Skill}"
    
    # Convert CamelCase to snake_case for the output directory
    output_dir="${skill_name,}"             # Convert first char to lowercase
    output_dir=$(echo "$output_dir" | sed 's/\([A-Z]\)/_\L\1/g')  # Convert remaining CamelCase to snake_case
    output_dir="${output_dir}_skill"        # Add _skill suffix
    
    # Create output directory
    mkdir -p "$OUTPUT_BASE/${output_dir}"
    
    echo "Processing $filename..."
    model2code \
        --input_filename "$scxml_file" \
        --model_filename "$MODEL_FILE" \
        --interface_filename "$INTERFACE_FILE" \
        --template_path "$TEMPLATE_PATH" \
        --output_path "$OUTPUT_BASE/${output_dir}" \
        --verbose_mode
done

echo "All skills processed."
