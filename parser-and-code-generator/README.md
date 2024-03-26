# SCXMLGEN
Tool to generate custom C++ files from SCXML

## Execution
To execute run:
```
scxmlgen --input_filename "filename.scxml" --model_filename "filename.scxml" --interface_filename "filename.scxml" --output_path "path/to/output/directory"
```
Default values are:
```
--input_filename        "in.scxml"
--model_filename        "./specification/examples/museum-guide/main-XML/full-model.xml"
--interface_filename    "./specification/examples/museum-guide/interface-definition-IDL/interfaces.xml"
--output_path           "./"
```