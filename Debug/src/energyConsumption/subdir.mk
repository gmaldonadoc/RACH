################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/energyConsumption/ue-energy-model.cpp \
../src/energyConsumption/ue-consumption-model.cpp \
../src/energyConsumption/ue-miletus-model.cpp \
../src/energyConsumption/ue-3gpp-model.cpp \
../src/energyConsumption/ue-lauridsen-model.cpp \
../src/energyConsumption/battery-model.cpp

OBJS += \
./src/energyConsumption/ue-energy-model.o \
./src/energyConsumption/ue-consumption-model.o \
./src/energyConsumption/ue-miletus-model.o \
./src/energyConsumption/ue-3gpp-model.o \
../src/energyConsumption/ue-lauridsen-model.o \
./src/energyConsumption/battery-model.o

CPP_DEPS += \
./src/energyConsumption/ue-energy-model.d \
./src/energyConsumption/ue-consumption-model.d \
./src/energyConsumption/ue-miletus-model.d \
./src/energyConsumption/ue-3gpp-model.d \
./src/energyConsumption/ue-lauridsen-model.d \
./src/energyConsumption/battery-model.d

# Each subdirectory must supply rules for building sources it contributes
src/energyConsumption/%.o: ../src/energyConsumption/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
