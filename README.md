# Map conversion - OpenDRIVE => Lanelet2

- Autoware uses Lanelet2 format for its ODD testing.
- We want more scenarios.
- ASAM OpenDRIVE dataset is a good option.

So, we need a way to convert OpenDRIVE to Lanelet2. We tried [CommonRoad Scenario Designer conversion](https://commonroad-scenario-designer.readthedocs.io/en/latest/details/open_drive/), the conversion generally worked, but the maps were containing way too many nodes, making it unusable on Autoware.

I'm attempting to fix that using 
