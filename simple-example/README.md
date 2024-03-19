to execute the simple behavior tree:
```
cd docker/
docker compose build
sudo xhost + 
docker compose up
```


to get the specification

```
cd specification
git init
git remote add -f origin https://github.com/convince-project/data-model
git config core.sparseCheckout true
echo "examples/museum-guide/environment-JANI" >> .git/info/sparse-checkout
echo "examples/museum-guide/environment-sim-URDF-SDF" >> .git/info/sparse-checkout
echo "examples/museum-guide/environment-XML" >> .git/info/sparse-checkout
echo "examples/museum-guide/interface-definition-IDL" >> .git/info/sparse-checkout
echo "examples/museum-guide/main-XML" >> .git/info/sparse-checkout
git pull origin main
```