#
# Component Makefile
#

COMPONENT_ADD_INCLUDEDIRS := esp-aws-iot/port/include esp-aws-iot/aws-iot-device-sdk-embedded-C/include

COMPONENT_SRCDIRS := esp-aws-iot/aws-iot-device-sdk-embedded-C/src port

# Check the submodule is initialised
COMPONENT_SUBMODULES := esp-aws-iot/aws-iot-device-sdk-embedded-C
