# TCL File Generated by Component Editor 17.0
# Thu Mar 01 15:03:17 MST 2018
# DO NOT MODIFY


# 
# upCounter "upCounter" v1.0
#  2018.03.01.15:03:17
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module upCounter
# 
set_module_property DESCRIPTION ""
set_module_property NAME upCounter
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR ""
set_module_property DISPLAY_NAME upCounter
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL upCounter
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE false
add_fileset_file upCounter.vhd VHDL PATH HallSensors/upCounter.vhd TOP_LEVEL_FILE


# 
# parameters
# 


# 
# display items
# 


# 
# connection point reset_sink
# 
add_interface reset_sink reset end
set_interface_property reset_sink associatedClock SystemClock
set_interface_property reset_sink synchronousEdges DEASSERT
set_interface_property reset_sink ENABLED true
set_interface_property reset_sink EXPORT_OF ""
set_interface_property reset_sink PORT_NAME_MAP ""
set_interface_property reset_sink CMSIS_SVD_VARIABLES ""
set_interface_property reset_sink SVD_ADDRESS_GROUP ""

add_interface_port reset_sink reset reset Input 1


# 
# connection point hallReading
# 
add_interface hallReading avalon end
set_interface_property hallReading addressUnits WORDS
set_interface_property hallReading associatedClock SystemClock
set_interface_property hallReading associatedReset reset_sink
set_interface_property hallReading bitsPerSymbol 8
set_interface_property hallReading burstOnBurstBoundariesOnly false
set_interface_property hallReading burstcountUnits WORDS
set_interface_property hallReading explicitAddressSpan 0
set_interface_property hallReading holdTime 0
set_interface_property hallReading linewrapBursts false
set_interface_property hallReading maximumPendingReadTransactions 0
set_interface_property hallReading maximumPendingWriteTransactions 0
set_interface_property hallReading readLatency 0
set_interface_property hallReading readWaitTime 1
set_interface_property hallReading setupTime 0
set_interface_property hallReading timingUnits Cycles
set_interface_property hallReading writeWaitTime 0
set_interface_property hallReading ENABLED true
set_interface_property hallReading EXPORT_OF ""
set_interface_property hallReading PORT_NAME_MAP ""
set_interface_property hallReading CMSIS_SVD_VARIABLES ""
set_interface_property hallReading SVD_ADDRESS_GROUP ""

add_interface_port hallReading readIn read Input 1
add_interface_port hallReading sendValue readdata Output 8
set_interface_assignment hallReading embeddedsw.configuration.isFlash 0
set_interface_assignment hallReading embeddedsw.configuration.isMemoryDevice 0
set_interface_assignment hallReading embeddedsw.configuration.isNonVolatileStorage 0
set_interface_assignment hallReading embeddedsw.configuration.isPrintableDevice 0


# 
# connection point SystemClock
# 
add_interface SystemClock clock end
set_interface_property SystemClock clockRate 0
set_interface_property SystemClock ENABLED true
set_interface_property SystemClock EXPORT_OF ""
set_interface_property SystemClock PORT_NAME_MAP ""
set_interface_property SystemClock CMSIS_SVD_VARIABLES ""
set_interface_property SystemClock SVD_ADDRESS_GROUP ""

add_interface_port SystemClock sysClk clk Input 1


# 
# connection point HallSensorInput
# 
add_interface HallSensorInput conduit end
set_interface_property HallSensorInput associatedClock ""
set_interface_property HallSensorInput associatedReset ""
set_interface_property HallSensorInput ENABLED true
set_interface_property HallSensorInput EXPORT_OF ""
set_interface_property HallSensorInput PORT_NAME_MAP ""
set_interface_property HallSensorInput CMSIS_SVD_VARIABLES ""
set_interface_property HallSensorInput SVD_ADDRESS_GROUP ""

add_interface_port HallSensorInput hallSensorIn export Input 1

