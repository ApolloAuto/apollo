
## Gen Vehicle Protocol Tool
	It's a convinent tool to let you quickly generate a nearly complete code for a new vehicle.
You only have to do is to have the dbc file (which is a communication protocol for the car, which is usually made by the vehicle integrated company), and write a less 10 lines config for generate an encode/decode `

## Dependency

Firstly, enter the apollo container environment
> aem enter

Secondly, put the lincoln.dbc into /apollo_workspace/output

## Config & Flags

```yml
# difine the lincoln.dbc absulte file path for your vehilce
dbc_file: /apollo_workspace/output/lincoln.dbc
# define extract the dbc file to generate the temp absulte file path
protocol_conf: /apollo_workspace/output/lincoln.yml
# define the lincoln car type
car_type: lincoln
# default: []
sender_list: []
# define in lincoln.dbc file AD ECU node name, which means "Apollo control lincoln vehicle msgs"
sender: MAB
# default: []
black_list: []
# default: false
use_demo_dbc: false

# the generated canbus vehicle code path
# defalut: /apollo_workspace/modules/canbus_vehicle
output_dir: /apollo_workspace/modules/canbus_vehicle
# the generated canbus conf path
# defalut: /apollo_workspace/output
output_canbus_conf_dir: /apollo_workspace/output
```

## Usage

The tool's input is :

* `vehicle dbc file`: like lincoln's dbc file, put it under this folder
* `generator tool config file`: for an example, a lincoln's is lincoln_conf.yml, detail you can see the example file of lincoln_conf.yml

Run:

*Notice: you need to run the following code in the container environment.*

> gen output/lincoln_conf.yml

## Tool Framework

* `gen.py` : a central control the overall generating progress and will call the scripts below
* `extract_dbc_meta.py`: extract dbc info to an internal generator tool used yaml config, which will include a protocol name, id, how many vars in a protocol, every var's name, type, byte start, bit_start, bit_len etc. When we have these info, we can automitally generate code as we wish.
* `gen_protoco_file.py`: generate a proto file for this vehicle, which is used to store parsed info from CAN frame using these generated code.
* `gen_protocols.py`: generate protocol code (encoding and decoding CAN frame)according to the extract dbc meta.
* `gen_vehicle_controller_and_manager.py`: generate vehicle controller and vehicle message manager according to our recent chassis canbus framework.
* `gen_canbus_conf.py`: generate vehicle canbus conf file.


## More Detail Tutorial

Please read this markdown file: [apollo_vehicle_adaption_tutorial_cn](../../../docs/11_Hardware%20Integration%20and%20Calibration/车辆适配/apollo_vehicle_adaption_tutorial_cn.md)