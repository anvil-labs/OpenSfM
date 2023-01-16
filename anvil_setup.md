# Anvil Setup

## Instructions

This is a loose collection of several of the manual commands that needed to be run in order to get the `temp.py` and `temp2.py` files running in the Docker container properly.

**In the shell**:

1. Move the image resources you want to use into the `data/` folder e.g. `data/odm_data_aukerman`.

2. Build the container.

```
docker build -t opensfm-python-runner .
```

3. Run the container in interactive mode.

```
docker run -it -v $(pwd):/source/OpenSfM -p 8000:8000 opensfm-python-runner
```

**In the container**:

1. Install the Node resources.

```
./viewer/node_modules.sh
```

2. Run the Python setup.

```
python3 setup.py build
```

3. Run with the provided data.

```
./bin/opensfm_run_all data/odm_data_aukerman
```

4. Serve the data.

```
python3 viewer/server.py -d data/odm_data_aukerman -p 8000
```

## Notes

- The `commands` package is not really being used but you can execute custom commands in there with `./bin/opensfm test_command` (for example).
