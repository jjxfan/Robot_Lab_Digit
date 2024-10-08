# robot_lab

robot_lab is an extension project based on Isaac Lab. It allows you to develop in an isolated environment, outside of the core Isaac Lab repository.

If you want to run policy in gazebo or real robot, please use [rl_sar](https://github.com/fan-ziqi/rl_sar) project.

Todo:

- [x] AMP training
- [ ] VAE training code
- [x] Sim to Sim transfer(Gazebo)
- [ ] Sim to Real transfer(Unitree A1)

[Click to discuss on Discord](https://discord.gg/vmVjkhVugU)

## Get Ready

You need to install `Isaac Lab`.

## Installation

Using a python interpreter that has Isaac Lab installed, install the library

```bash
python -m pip install -e ./exts/robot_lab
```

## Try examples

FFTAI GR1T1

```bash
# Train
python scripts/rsl_rl/train.py --task RobotLab-Isaac-Velocity-Flat-FFTAI-GR1T1-v0 --headless
# Play
python scripts/rsl_rl/play.py --task RobotLab-Isaac-Velocity-Flat-FFTAI-GR1T1-v0
```

Anymal D

```bash
# Train
python scripts/rsl_rl/train.py --task RobotLab-Isaac-Velocity-Flat-Anymal-D-v0 --headless
# Play
python scripts/rsl_rl/play.py --task RobotLab-Isaac-Velocity-Flat-Anymal-D-v0
```

Unitree A1

```bash
# Train
python scripts/rsl_rl/train.py --task RobotLab-Isaac-Velocity-Flat-Unitree-A1-v0 --headless
# Play
python scripts/rsl_rl/play.py --task RobotLab-Isaac-Velocity-Flat-Unitree-A1-v0
```

Unitree Go2W (Unvalible for now)

```bash
# Train
python scripts/rsl_rl/train.py --task RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0 --headless
# Play
python scripts/rsl_rl/play.py --task RobotLab-Isaac-Velocity-Flat-Unitree-Go2W-v0
```

Unitree H1

```bash
# Train
python scripts/rsl_rl/train.py --task RobotLab-Isaac-Velocity-Flat-Unitree-H1-v0 --headless
# Play
python scripts/rsl_rl/play.py --task RobotLab-Isaac-Velocity-Flat-Unitree-H1-v0
```

The above configs are flat, you can change Flat to Rough

**Note**

* Record video of a trained agent (requires installing `ffmpeg`), add `--video --video_length 200`
* Play/Train with 32 environments, add `--num_envs 32`
* Play on specific folder or checkpoint, add `--load_run run_folder_name --checkpoint model.pt`
* Resume training from folder or checkpoint, add `--resume --load_run run_folder_name --checkpoint model.pt`

## AMP training

The code for AMP training refers to [AMP_for_hardware](https://github.com/Alescontrela/AMP_for_hardware)

Unitree A1

```bash
# Retarget motion files
python exts/robot_lab/amp_utils/scripts/retarget_kp_motions.py
# Replay AMP data
python scripts/rsl_rl/replay_amp_data.py --task RobotLab-Isaac-Velocity-Flat-Amp-Unitree-A1-v0
# Train
python scripts/rsl_rl/train_amp.py --task RobotLab-Isaac-Velocity-Flat-Amp-Unitree-A1-v0 --headless
# Play
python scripts/rsl_rl/play_amp.py --task RobotLab-Isaac-Velocity-Flat-Amp-Unitree-A1-v0
```

## Add your own robot

For example, to generate Unitree A1 usd file:

```bash
python scripts/tools/convert_urdf.py a1.urdf exts/robot_lab/data/Robots/Unitree/A1/a1.usd  --merge-join
```

Check [import_new_asset](https://docs.robotsfan.com/isaaclab/source/how-to/import_new_asset.html) for detail

## Tensorboard

To view tensorboard, run:

```bash
tensorboard --logdir=logs
```

## Code formatting

A pre-commit template is given to automatically format the code.

To install pre-commit:

```bash
pip install pre-commit
```

Then you can run pre-commit with:

```bash
pre-commit run --all-files
```

## Citation

Please cite the following if you use this code or parts of it:

```
@software{fan-ziqi2024robot_lab,
  author = {fan-ziqi},
  title = {{robot_lab: An extension project based on Isaac Lab.}},
  url = {https://github.com/fan-ziqi/robot_lab},
  year = {2024}
}
```
