################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drx/drx-manager.cpp 

OBJS += \
./src/drx/drx-manager.o 

CPP_DEPS += \
./src/drx/drx-manager.d 

# Each subdirectory must supply rules for building sources it contributes
src/drx/%.o: ../src/drx/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
