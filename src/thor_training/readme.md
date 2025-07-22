LeRobot Training Guide for Thor Arm
This guide walks you through the process of taking your recorded thor_dataset.hdf5 file and using it to train a new AI policy for your robot arm.

Step 1: Setting Up the Training Environment
It is crucial to create a dedicated Python virtual environment for LeRobot to avoid conflicts with your ROS 2 installation or other projects.

Clone the LeRobot Repository:
First, find a suitable location on your computer (outside of your ROS 2 workspace) and clone the official LeRobot GitHub repository.

git clone https://github.com/huggingface/lerobot.git
cd lerobot

Create and Activate a Virtual Environment:
We will use conda, which is recommended for managing PyTorch dependencies.

# Create a new environment with Python 3.10
conda create -n lerobot python=3.10

# Activate the new environment
conda activate lerobot

Install LeRobot and Dependencies:
Now, install the LeRobot library and its dependencies, including PyTorch with GPU support (if you have an NVIDIA GPU).

# Install PyTorch (this command is for CUDA 12.1, check the PyTorch website for your specific CUDA version)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# Install the LeRobot library in editable mode
pip install -e .

Install h5py:
You'll need this library to read the dataset file.

pip install h5py

Step 2: Prepare Your Dataset and Training Script
Copy Your Dataset:
Move the thor_dataset_... .hdf5 file you created into the main lerobot directory you just cloned.

Create the Training Script:
Inside the lerobot directory, create a new file named train_thor_policy.py and paste the following code into it.

# ========================================================================
# File: lerobot/train_thor_policy.py
# ========================================================================
# This script loads the custom Thor dataset and trains an ACT policy.
#
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.common.training.trainer import Trainer
from lerobot.common.training.utils import get_log_output_dir

def main():
    # --- 1. Load Your Custom Dataset ---
    # Replace with the actual name of your dataset file
    dataset_path = "thor_dataset_20250722_2245.hdf5" 
    print(f"Loading dataset from: {dataset_path}")
    dataset = LeRobotDataset(dataset_path)

    # --- 2. Configure the AI Policy ---
    # We will use the ACT (Action Chunking with Transformers) policy,
    # which is a powerful model for imitation learning.

    # This configuration needs to match the data we recorded.
    config = ACTConfig(
        # Observation space
        in_shape={
            "observation.top_image": (3, 480, 640), # (Channels, Height, Width)
            "observation.base_image": (3, 480, 640),
            "observation.joint_states": (6,),
            "observation.ee_pose": (6,)
        },
        # Action space (7-DoF: X,Y,Z, R,P,Y, Gripper)
        out_shape={"action": (7,)},
    )

    # --- 3. Initialize the Policy Model ---
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    policy = ACTPolicy(config, dataset_stats=dataset.stats).to(device)

    # --- 4. Configure and Start the Trainer ---
    output_dir = get_log_output_dir()
    print(f"Training outputs will be saved to: {output_dir}")

    trainer = Trainer(
        policy=policy,
        dataset=dataset,
        output_dir=output_dir,
        # Training parameters (you can tune these)
        num_epochs=500,
        batch_size=32,
        learning_rate=1e-4,
    )

    print("Starting training...")
    trainer.train()
    print("Training complete!")

if __name__ == "__main__":
    main()


Step 3: Run the Training
Activate the Environment: Make sure your lerobot conda environment is still active.

conda activate lerobot

Edit the Script: Open train_thor_policy.py and change the dataset_path variable to match the exact filename of your dataset.

Start Training: Run the script from your terminal.

python train_thor_policy.py

What to Expect
The script will first load your dataset and print some statistics.

It will then initialize the ACT policy model.

Finally, the training loop will begin. You will see a progress bar showing the training loss decreasing over many "epochs." This process can take anywhere from a few minutes to several hours, depending on the size of your dataset and the power of your GPU.

When training is complete, a new directory will be created (e.g., outputs/train/YYYY-MM-DD/HH-MM-SS). Inside this directory, you will find the most important file: model.pth.

This model.pth file is your trained AI policy. It contains all the learned "knowledge" from your demonstrations. In the final phase, we will create an inference node that loads this file to control the robot autonomously.