# Bundled Gazebo models

The environment assets in `models/environments/` are copied from the model library already used by the `sim_agent` configurator. The cardboard box, table, and brick box originate from the [Open Robotics Gazebo model database](https://github.com/osrf/gazebo_models), licensed under [CC BY 3.0](https://creativecommons.org/licenses/by/3.0/). Their original `model.config` files retain author credits. The bundled `Table/model.sdf` changes four visual names to make them unique under SDFormat validation; geometry and appearance are unchanged.

The Office Chair's original geometry is by **zhixson** and is licensed under [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). Its `model.config` retains the original source link and credits Cole Biesemeyer for the SDF adaptation.

The `x500_base` resources come from the PX4 model already used by the drone configurator. Its original `LICENSE` file is included in `models/x500_base/`.

The floor, wall, and brick textures preserve the configurator's room appearance. The model and texture files are bundled solely to resolve local `model://` references during offline tests; no network model download is required.
