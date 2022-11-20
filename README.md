HorizonRPCServerExample
=======================

A simple example to provide sensor data / traffic data and generic data to "Horizon - Your Portable Glass Cockpit" ( https://helios-avionics.com ) via JSON-RPC.

It utilizes the JSON-RPC 2.0 implementation from https://github.com/wolfgang1991/CommonLibraries . This way it is not needed to write any JSON by hand.
For reference purposes it is possible to print all JSON-RPC traffic to stdout by compiling in debug mode (DEBUG flag set when executing make).
The JSON-RPC traffic uses UTF-8 and is zlib compressed. A specification can be found here: https://www.jsonrpc.org/specification

Generic data can be indicated in Horizon using the BarPanel or the TextPanel (Menu -> Setup -> Panel Setup). The key for generic values (field name in JSON object) needs to be specified in the respective panel setup (see also GenericValues in RPCSensorAPI.h ).

Please check the doxygen comments in RPCSensorAPI.h fpr further documentation.
