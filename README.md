# Lidar Point Cloud Compression

## Setup

### Requirements

This setup guide is for Linux. Windows should work too, but does require a few configurations to make it work.

- Laptop with NVIDIA GPU
- Docker
- KITTI Dataset
- VSCode (not needed, but allows for easier development)

### Initializing ENV

If you are on VSCode, there should be a prompt to let you build the container based on the `docker-compose.yaml` and the `Dockerfile`.

You can also build the container via `docker compose up --build -d`.

--- 
**NOTE** : Under `volumes`, theres is the line

```
    - /mnt/d/Datasets/KITTIDemo:/workspace/dataset:rw
```

`/mnt/d/Datasets/KITTIDemo` should be replaced with wherever your KITTI dataset is. `KITTIDemo` is simply the KITTI dataset, but with only 100 examples.

---

Once the container has been built, log into the execution shell via `docker exec -it lpcc-env bash`. Once in and in the `/workspace` directory perform the following steps :

1. Set up Point Pillars
```
$ cd pointpillars
$ pip install -r requirements.txt
$ python setup.py build_ext --inplace
$ pip install .
```

2. Navigate back to `/workspace`

3. Set up RCPCC
```
$ ./scripts/rcpcc/build.sh
```

## Running Scripts

The main script to run is `./scripts/utils/run_pipeline.sh`. This script will perform the following:

1. Run the KITTI dataset through the RCPCC compression and decompression
    - we will call this dataset RCPCC_KITTI
2. Preprocess the RCPCC_KITTI and raw KITTI dataset with `./pointpillars`'s `pre_process_kitti.py`
3. Evaluate the Point Pillars model with the processed RCPCC_KITTI

Each step in the process above can be individually called via

1. `./scripts/rcpcc/process.sh`
2. `./scripts/pointpillars/preprocess.sh`
3. `./scripts/pointpillars/evaluate.sh`

To make sure that scripts **2** and **3** run on RCPCC_KITTI, add `-p rcpcc` to the end.

Ex:
```
$ ./scripts/pointpillars/preprocess.sh -p rcpcc
```
