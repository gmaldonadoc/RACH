################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/protocolStack/mac/pdcch-scheduler/pdcch-scheduler.cpp

OBJS += \
./src/protocolStack/mac/pdcch-scheduler/pdcch-scheduler.o

CPP_DEPS += \
./src/protocolStack/mac/pdcch-scheduler/pdcch-scheduler.d

# Each subdirectory must supply rules for building sources it contributes
src/protocolStack/mac/pdcch-scheduler/%.o: ../src/protocolStack/mac/pdcch-scheduler/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '