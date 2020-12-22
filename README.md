# CustomRLEnvs

This repository contains custom Reinforcement Learning environments created using PyBullet as well as their implementation with A2C. Included environments:
 - HumanoidBiped
 - Simple two links
 
Run `HumanoidBipedA2C.py` and `SimpleEnvA2C.py` to check out the tranining process.

## TO-DO
Need to do the following for improving the results:
 - Modify reward function
 
  - Add negative reward for Collsion/contact among limbs
  - Add positive reward for contact of feet with the floor/ground
