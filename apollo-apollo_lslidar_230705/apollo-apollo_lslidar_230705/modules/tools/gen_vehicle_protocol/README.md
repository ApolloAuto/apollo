
## Gen Vehicle Protocol Tool
	It's a convinent tool to let you quickly generate a nearly complete code for a new vehicle.
You only have to do is to have the dbc file (which is a communication protocol for the car, which is usually made by the vehicle integrated company), and write a less 10 lines config for generate an encode/decode `

## Dependency

> sudo pip install pyyaml

## Usage:

The tool's input is :

* `vehicle dbc file `: like lincoln's dbc file, put it under this folder
* `generator tool config file`: for an example, a lincoln's is lincoln_conf.yml, detail you can see the example file of lincoln_conf.yml

Run:

> python gen.py lincoln_conf.yml

## Tool Framework

* `gen.py` : a central control the overall generating progress and will call the scripts below
* `extract_dbc_meta.py`: extract dbc info to an internal generator tool used yaml config, which will include a protocol name, id, how many vars in a protocol, every var's name, type, byte start, bit_start, bit_len etc. When we have these info, we can automitally generate code as we wish.
* `gen_protoco_file.py`: generate a proto file for this vehicle, which is used to store parsed info from CAN frame using these generated code.
* `gen_protocols.py`: generate protocol code (encoding and decoding CAN frame)according to the extract dbc meta.
* `gen_vehicle_controller_and_manager`: generate vehicle controller and vehicle message manager according to our recent chassis canbus framework.


